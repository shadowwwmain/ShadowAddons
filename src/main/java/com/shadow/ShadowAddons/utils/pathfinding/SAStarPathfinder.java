package com.shadow.ShadowAddons.utils.pathfinding;

import net.minecraft.block.Block;
import net.minecraft.block.BlockLadder;
import net.minecraft.block.BlockStairs;
import net.minecraft.block.material.Material;
import net.minecraft.client.Minecraft;
import net.minecraft.client.renderer.GlStateManager;
import net.minecraft.entity.player.EntityPlayer;
import net.minecraft.init.Blocks;
import net.minecraft.util.BlockPos;
import net.minecraft.world.World;
import net.minecraftforge.client.event.RenderWorldLastEvent;
import net.minecraftforge.common.MinecraftForge;
import net.minecraftforge.fml.common.eventhandler.SubscribeEvent;
import org.lwjgl.opengl.GL11;

import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

public class SAStarPathfinder {

    // Configuration
    public static class Config {
        // Bump these up so very long paths can be found:
        public static int MAX_SEARCH_RADIUS = 300;   // allow up to ~300‐block radius
        public static int MAX_SEARCH_DEPTH  = 200;   // allow deeper expansion
        public static int MAX_JUMP_HEIGHT   = 3;
        public static int MAX_FALL_DISTANCE = 20;    // can fall up to 20 blocks safely
        public static boolean AVOID_LAVA    = true;
        public static boolean ALLOW_SWIMMING  = true;
        public static boolean ALLOW_CLIMBING  = true;
        public static HeuristicType HEURISTIC_TYPE = HeuristicType.MANHATTAN;

        // Debug settings (keep as you like)
        public static boolean DEBUG_ENABLED    = true;
        public static boolean SHOW_PATH        = true;
        public static boolean SHOW_NODES       = true;
        public static boolean SHOW_COSTS       = false;
        public static boolean SHOW_SEARCH_AREA = false;
        public static boolean SHOW_METRICS     = true;
    }

    public enum HeuristicType {
        MANHATTAN, EUCLIDEAN, DIAGONAL
    }

    // Node class for A* algorithm
    public static class PathNode implements Comparable<PathNode> {
        public final BlockPos pos;
        public final PathNode parent;
        public final double gCost; // Distance from start
        public final double hCost; // Heuristic distance to goal
        public final double fCost; // Total cost
        public final MovementType movementType;

        public PathNode(BlockPos pos, PathNode parent, double gCost, double hCost, MovementType movementType) {
            this.pos = pos;
            this.parent = parent;
            this.gCost = gCost;
            this.hCost = hCost;
            this.fCost = gCost + hCost;
            this.movementType = movementType;
        }

        @Override
        public int compareTo(PathNode other) {
            int result = Double.compare(this.fCost, other.fCost);
            if (result == 0) {
                result = Double.compare(this.hCost, other.hCost);
            }
            return result;
        }

        @Override
        public boolean equals(Object obj) {
            return obj instanceof PathNode && ((PathNode) obj).pos.equals(this.pos);
        }

        @Override
        public int hashCode() {
            return pos.hashCode();
        }
    }

    public enum MovementType {
        WALK(1.0), JUMP(1.3), CLIMB(1.5), SWIM(2.0), FALL(0.8), DIAGONAL(1.4);

        public final double cost;

        MovementType(double cost) {
            this.cost = cost;
        }
    }

    // Pathfinding result
    public static class PathResult {
        public final List<BlockPos> path;
        public final boolean success;
        public final long searchTime;
        public final int nodesExplored;
        public final double pathLength;
        public final String failureReason;

        public PathResult(List<BlockPos> path, boolean success, long searchTime,
                          int nodesExplored, double pathLength, String failureReason) {
            this.path = path;
            this.success = success;
            this.searchTime = searchTime;
            this.nodesExplored = nodesExplored;
            this.pathLength = pathLength;
            this.failureReason = failureReason;
        }
    }

    // Instance variables
    private final World world;
    private final Map<BlockPos, Block> blockCache = new ConcurrentHashMap<>();
    private final Set<BlockPos> failedPaths = new HashSet<>();
    private PathResult lastResult;
    private final DebugRenderer debugRenderer;

    // Singleton instance for API access
    private static SAStarPathfinder instance;

    public SAStarPathfinder(World world) {
        this.world = world;
        this.debugRenderer = new DebugRenderer();
        MinecraftForge.EVENT_BUS.register(debugRenderer);
        instance = this;
    }

    // API Access
    public static SAStarPathfinder getInstance() {
        if (instance == null) {
            throw new IllegalStateException("SAStarPathfinder not initialized! Create instance first.");
        }
        return instance;
    }

    public static void initialize(World world) {
        new SAStarPathfinder(world);
    }

    // Main pathfinding method
    public PathResult findPath(BlockPos start, BlockPos goal) {
        return findPath(start, goal, Config.MAX_SEARCH_RADIUS);
    }

    public PathResult findPath(BlockPos start, BlockPos goal, int maxRadius) {
        long startTime = System.nanoTime();

        // Check if this path has failed before
        if (failedPaths.contains(start) || failedPaths.contains(goal)) {
            return new PathResult(new ArrayList<>(), false, 0, 0, 0, "Previously failed path");
        }

        // Validate start and goal
        if (!isValidPosition(start) || !isValidPosition(goal)) {
            return new PathResult(new ArrayList<>(), false, 0, 0, 0, "Invalid start or goal position");
        }

        if (start.equals(goal)) {
            return new PathResult(Arrays.asList(start), true, 0, 1, 0, null);
        }

        // Initialize A* algorithm
        PriorityQueue<PathNode> openSet = new PriorityQueue<>();
        Set<BlockPos> closedSet = new HashSet<>();
        Map<BlockPos, PathNode> allNodes = new HashMap<>();

        PathNode startNode = new PathNode(start, null, 0, calculateHeuristic(start, goal), MovementType.WALK);
        openSet.add(startNode);
        allNodes.put(start, startNode);

        int nodesExplored = 0;

        while (!openSet.isEmpty() && nodesExplored < maxRadius * maxRadius) {
            PathNode current = openSet.poll();
            closedSet.add(current.pos);
            nodesExplored++;

            // Check if we reached the goal
            if (current.pos.equals(goal)) {
                List<BlockPos> path = reconstructPath(current);
                double pathLength = calculatePathLength(path);
                long searchTime = (System.nanoTime() - startTime) / 1000000; // ms

                PathResult result = new PathResult(path, true, searchTime, nodesExplored, pathLength, null);
                lastResult = result;
                debugRenderer.updatePath(path,
                        new ArrayList<>(closedSet),
                        new ArrayList<>(openSet.stream().map(n -> n.pos).collect(ArrayList::new, ArrayList::add, ArrayList::addAll))
                );
                return result;
            }

            // Explore neighbors
            for (PathNode neighbor : getNeighbors(current, goal)) {
                if (closedSet.contains(neighbor.pos)) continue;

                PathNode existingNode = allNodes.get(neighbor.pos);
                if (existingNode == null || neighbor.gCost < existingNode.gCost) {
                    allNodes.put(neighbor.pos, neighbor);
                    openSet.add(neighbor);
                }
            }
        }

        // Path not found
        failedPaths.add(start);
        long searchTime = (System.nanoTime() - startTime) / 1000000;
        PathResult result = new PathResult(new ArrayList<>(), false, searchTime, nodesExplored, 0, "No path found");
        lastResult = result;
        return result;
    }

    // Get valid neighbors for current node
    private List<PathNode> getNeighbors(PathNode current, BlockPos goal) {
        List<PathNode> neighbors = new ArrayList<>();
        BlockPos pos = current.pos;

        // 8-directional movement
        int[] dx = {-1, -1, -1, 0, 0, 1, 1, 1};
        int[] dz = {-1,  0,  1, -1,1, -1, 0,  1};

        for (int i = 0; i < 8; i++) {
            BlockPos newPos = pos.add(dx[i], 0, dz[i]);

            // Walk on same level or 1-block step
            tryMovement(neighbors, current, newPos, goal, MovementType.WALK);
            // Jump up
            tryMovement(neighbors, current, newPos.up(), goal, MovementType.JUMP);
            // Fall down
            tryMovement(neighbors, current, newPos.down(), goal, MovementType.FALL);

            // Climbing
            if (Config.ALLOW_CLIMBING) {
                tryClimbing(neighbors, current, newPos, goal);
            }

            // Swimming
            if (Config.ALLOW_SWIMMING) {
                trySwimming(neighbors, current, newPos, goal);
            }
        }

        return neighbors;
    }

    private void tryMovement(List<PathNode> neighbors, PathNode current, BlockPos pos, BlockPos goal, MovementType movementType) {
        if (!isValidMovement(current.pos, pos, movementType)) return;

        double movementCost = getMovementCost(current.pos, pos, movementType);
        double gCost = current.gCost + movementCost;
        double hCost = calculateHeuristic(pos, goal);

        neighbors.add(new PathNode(pos, current, gCost, hCost, movementType));
    }

    private void tryClimbing(List<PathNode> neighbors, PathNode current, BlockPos pos, BlockPos goal) {
        for (int y = 1; y <= Config.MAX_JUMP_HEIGHT; y++) {
            BlockPos climbPos = pos.up(y);
            if (isClimbable(pos.up(y - 1)) && isValidPosition(climbPos)) {
                double movementCost = getMovementCost(current.pos, climbPos, MovementType.CLIMB);
                double gCost = current.gCost + movementCost;
                double hCost = calculateHeuristic(climbPos, goal);
                neighbors.add(new PathNode(climbPos, current, gCost, hCost, MovementType.CLIMB));
            }
        }
    }

    private void trySwimming(List<PathNode> neighbors, PathNode current, BlockPos pos, BlockPos goal) {
        if (isWater(pos) && isValidPosition(pos)) {
            double movementCost = getMovementCost(current.pos, pos, MovementType.SWIM);
            double gCost = current.gCost + movementCost;
            double hCost = calculateHeuristic(pos, goal);
            neighbors.add(new PathNode(pos, current, gCost, hCost, MovementType.SWIM));
        }
    }

    // Movement validation
    private boolean isValidMovement(BlockPos from, BlockPos to, MovementType movementType) {
        if (!isValidPosition(to)) return false;
        int deltaY = to.getY() - from.getY();

        switch (movementType) {
            case WALK:
                return canWalkTo(from, to);
            case JUMP:
                return deltaY > 0 && deltaY <= Config.MAX_JUMP_HEIGHT && canJumpTo(from, to);
            case FALL:
                return deltaY < 0 && canFallTo(from, to);
            case CLIMB:
                return deltaY > 0 && canClimbTo(from, to);
            case SWIM:
                return canSwimTo(from, to);
            default:
                return false;
        }
    }

    private boolean canWalkTo(BlockPos from, BlockPos to) {
        int deltaY = to.getY() - from.getY();

        // Flat walk
        if (deltaY == 0) {
            return isWalkable(to) && !hasObstacle(from, to, 2);
        }

        // Step up one block as “stairs”
        if (deltaY == 1) {
            if (!isValidPosition(to)) return false;          // block must be empty
            if (!isSolid(getBlock(to.down()))) return false; // solid underfoot
            if (isSolid(getBlock(to))) return false;         // space must be empty
            if (isSolid(getBlock(to.up()))) return false;    // headspace must be empty
            return !hasObstacle(from, to, 2);
        }

        return false;
    }

    private boolean canJumpTo(BlockPos from, BlockPos to) {
        int deltaY = to.getY() - from.getY();
        return deltaY <= Config.MAX_JUMP_HEIGHT && isWalkable(to) && !hasObstacle(from, to, deltaY + 2);
    }

    private boolean canFallTo(BlockPos from, BlockPos to) {
        int deltaY = to.getY() - from.getY();
        if (deltaY >= 0) return false; // only downward

        if (Math.abs(deltaY) > Config.MAX_FALL_DISTANCE) return false;
        if (!isWalkable(to)) return false; // must land on walkable spot

        // Ensure all blocks between from.y-1 down to to.y are non-solid
        BlockPos check = from.down();
        while (check.getY() >= to.getY()) {
            if (isSolid(getBlock(check))) {
                return false;
            }
            check = check.down();
        }
        return true;
    }

    private boolean canClimbTo(BlockPos from, BlockPos to) {
        return isClimbable(to) || (isWalkable(to) && isClimbable(from.up()));
    }

    private boolean canSwimTo(BlockPos from, BlockPos to) {
        return isWater(to) && !hasObstacle(from, to, 2);
    }

    // Block type checking
    private boolean isValidPosition(BlockPos pos) {
        if (pos.getY() < 0 || pos.getY() > 256) return false;
        return !isDangerous(pos) && (isWalkable(pos) || isWater(pos) || isClimbable(pos));
    }

    private boolean isWalkable(BlockPos pos) {
        Block below = getBlock(pos.down());
        Block at    = getBlock(pos);
        Block above = getBlock(pos.up());
        return isSolid(below) && !isSolid(at) && !isSolid(above) && !isLava(pos);
    }

    private boolean isWater(BlockPos pos) {
        return getBlock(pos).getMaterial() == Material.water;
    }

    private boolean isLava(BlockPos pos) {
        return getBlock(pos).getMaterial() == Material.lava;
    }

    private boolean isClimbable(BlockPos pos) {
        Block block = getBlock(pos);
        return block instanceof BlockLadder || block == Blocks.vine || block == Blocks.waterlily;
    }

    private boolean isSolid(Block block) {
        return block != null && block.isFullBlock() && block.getMaterial().isSolid();
    }

    private boolean isDangerous(BlockPos pos) {
        if (Config.AVOID_LAVA && isLava(pos)) return true;
        Block block = getBlock(pos);
        return block == Blocks.fire || block == Blocks.cactus || block.getMaterial() == Material.lava;
    }

    private boolean hasObstacle(BlockPos from, BlockPos to, int height) {
        int dx = Integer.signum(to.getX() - from.getX());
        int dz = Integer.signum(to.getZ() - from.getZ());
        BlockPos current = from;

        while (!current.equals(to)) {
            for (int y = 0; y < height; y++) {
                if (isSolid(getBlock(current.up(y)))) {
                    return true;
                }
            }
            current = current.add(dx, 0, dz);
        }
        return false;
    }

    // Cost calculation
    private double getMovementCost(BlockPos from, BlockPos to, MovementType movementType) {
        double baseCost = movementType.cost;
        double distance = from.distanceSq(to);

        double terrainModifier = 1.0;
        if (isWater(to)) terrainModifier *= 1.5;
        if (getBlock(to.down()) instanceof BlockStairs) terrainModifier *= 0.9;
        if (isDiagonal(from, to)) terrainModifier *= MovementType.DIAGONAL.cost;
        if (nearDanger(to)) terrainModifier *= 2.0;

        return baseCost * Math.sqrt(distance) * terrainModifier;
    }

    private boolean isDiagonal(BlockPos from, BlockPos to) {
        return Math.abs(from.getX() - to.getX()) > 0 && Math.abs(from.getZ() - to.getZ()) > 0;
    }

    private boolean nearDanger(BlockPos pos) {
        for (int x = -1; x <= 1; x++) {
            for (int z = -1; z <= 1; z++) {
                if (isDangerous(pos.add(x, 0, z)) || isDangerous(pos.add(x, -1, z))) {
                    return true;
                }
            }
        }
        return false;
    }

    // Heuristic calculation
    private double calculateHeuristic(BlockPos from, BlockPos to) {
        switch (Config.HEURISTIC_TYPE) {
            case MANHATTAN:
                return Math.abs(from.getX() - to.getX())
                        + Math.abs(from.getY() - to.getY())
                        + Math.abs(from.getZ() - to.getZ());
            case EUCLIDEAN:
                return Math.sqrt(from.distanceSq(to));
            case DIAGONAL:
                int dx = Math.abs(from.getX() - to.getX());
                int dy = Math.abs(from.getY() - to.getY());
                int dz = Math.abs(from.getZ() - to.getZ());
                return Math.max(dx, dz) + Math.abs(dy - Math.max(dx, dz)) * 0.414;
            default:
                return from.distanceSq(to);
        }
    }

    // Path reconstruction
    private List<BlockPos> reconstructPath(PathNode goalNode) {
        List<BlockPos> path = new ArrayList<>();
        PathNode current = goalNode;
        while (current != null) {
            path.add(0, current.pos);
            current = current.parent;
        }
        return optimizePath(path);
    }

    private List<BlockPos> optimizePath(List<BlockPos> path) {
        if (path.size() <= 2) return path;

        List<BlockPos> optimized = new ArrayList<>();
        optimized.add(path.get(0));
        int i = 0;
        while (i < path.size() - 1) {
            int farthest = i + 1;
            for (int j = i + 2; j < path.size(); j++) {
                if (hasDirectPath(path.get(i), path.get(j))) {
                    farthest = j;
                } else {
                    break;
                }
            }
            optimized.add(path.get(farthest));
            i = farthest;
        }
        return optimized;
    }

    private boolean hasDirectPath(BlockPos from, BlockPos to) {
        return !hasObstacle(from, to, 2);
    }

    private double calculatePathLength(List<BlockPos> path) {
        double length = 0;
        for (int i = 1; i < path.size(); i++) {
            length += Math.sqrt(path.get(i - 1).distanceSq(path.get(i)));
        }
        return length;
    }

    // Block caching for performance
    private Block getBlock(BlockPos pos) {
        return blockCache.computeIfAbsent(pos, p -> world.getBlockState(p).getBlock());
    }

    public void clearCache() {
        blockCache.clear();
        failedPaths.clear();
    }

    // API Methods
    public PathResult getLastResult() {
        return lastResult;
    }

    public void setDebugEnabled(boolean enabled) {
        Config.DEBUG_ENABLED = enabled;
    }

    public boolean isDebugEnabled() {
        return Config.DEBUG_ENABLED;
    }

    public void toggleDebugOverlay(String overlay) {
        switch (overlay.toLowerCase()) {
            case "path":
                Config.SHOW_PATH = !Config.SHOW_PATH;
                break;
            case "nodes":
                Config.SHOW_NODES = !Config.SHOW_NODES;
                break;
            case "costs":
                Config.SHOW_COSTS = !Config.SHOW_COSTS;
                break;
            case "area":
                Config.SHOW_SEARCH_AREA = !Config.SHOW_SEARCH_AREA;
                break;
            case "metrics":
                Config.SHOW_METRICS = !Config.SHOW_METRICS;
                break;
        }
    }

    // Debug Renderer
    public class DebugRenderer {
        private List<BlockPos> currentPath = new ArrayList<>();
        private List<BlockPos> closedNodes = new ArrayList<>();
        private List<BlockPos> openNodes = new ArrayList<>();

        public void updatePath(List<BlockPos> path, List<BlockPos> closed, List<BlockPos> open) {
            this.currentPath = new ArrayList<>(path);
            this.closedNodes = new ArrayList<>(closed);
            this.openNodes = new ArrayList<>(open);
        }

        @SubscribeEvent
        public void onRenderWorld(RenderWorldLastEvent event) {
            if (!Config.DEBUG_ENABLED) return;

            EntityPlayer player = Minecraft.getMinecraft().thePlayer;
            if (player == null) return;

            double x = player.lastTickPosX + (player.posX - player.lastTickPosX) * event.partialTicks;
            double y = player.lastTickPosY + (player.posY - player.lastTickPosY) * event.partialTicks;
            double z = player.lastTickPosZ + (player.posZ - player.lastTickPosZ) * event.partialTicks;

            GlStateManager.pushMatrix();
            GlStateManager.translate(-x, -y, -z);
            GlStateManager.disableTexture2D();
            GlStateManager.disableLighting();
            GlStateManager.enableBlend();

            // Render path
            if (Config.SHOW_PATH && !currentPath.isEmpty()) {
                renderPath();
            }

            // Render nodes
            if (Config.SHOW_NODES) {
                renderNodes();
            }

            // Render metrics
            if (Config.SHOW_METRICS && lastResult != null) {
                renderMetrics();
            }

            GlStateManager.enableTexture2D();
            GlStateManager.enableLighting();
            GlStateManager.disableBlend();
            GlStateManager.popMatrix();
        }

        private void renderPath() {
            GL11.glLineWidth(3.0f);
            GL11.glColor3f(0.0f, 1.0f, 0.0f); // Green for path
            GL11.glBegin(GL11.GL_LINE_STRIP);
            for (BlockPos pos : currentPath) {
                GL11.glVertex3d(pos.getX() + 0.5, pos.getY() + 0.5, pos.getZ() + 0.5);
            }
            GL11.glEnd();
        }

        private void renderNodes() {
            // Closed nodes (red)
            GL11.glColor3f(1.0f, 0.0f, 0.0f);
            for (BlockPos pos : closedNodes) {
                renderNodeBox(pos, 0.1f);
            }
            // Open nodes (blue)
            GL11.glColor3f(0.0f, 0.0f, 1.0f);
            for (BlockPos pos : openNodes) {
                renderNodeBox(pos, 0.1f);
            }
        }

        private void renderNodeBox(BlockPos pos, float size) {
            double x = pos.getX() + 0.5 - size;
            double y = pos.getY() + 0.5 - size;
            double z = pos.getZ() + 0.5 - size;
            double s = size * 2;

            GL11.glBegin(GL11.GL_QUADS);
            GL11.glVertex3d(x, y, z);
            GL11.glVertex3d(x + s, y, z);
            GL11.glVertex3d(x + s, y + s, z);
            GL11.glVertex3d(x, y + s, z);
            GL11.glEnd();
        }

        private void renderMetrics() {
            if (lastResult.success) {
                GL11.glColor3f(0.0f, 1.0f, 0.0f); // Green for success
            } else {
                GL11.glColor3f(1.0f, 0.0f, 0.0f); // Red for failure
            }
            EntityPlayer player = Minecraft.getMinecraft().thePlayer;
            BlockPos playerPos = player.getPosition();
            renderNodeBox(playerPos.up(3), 0.2f);
        }
    }
}
