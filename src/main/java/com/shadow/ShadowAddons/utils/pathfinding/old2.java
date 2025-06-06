package com.shadow.ShadowAddons.utils.pathfinding;

import com.shadow.ShadowAddons.utils.RotationsUtils.PathFindRotation;
import net.minecraft.block.*;
import net.minecraft.block.material.Material;
import net.minecraft.block.state.IBlockState;
import net.minecraft.client.Minecraft;
import net.minecraft.entity.EntityLivingBase;
import net.minecraft.init.Blocks;
import net.minecraft.util.*;
import net.minecraft.world.World;
import java.util.*;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.atomic.AtomicReference;
import net.minecraft.client.settings.KeyBinding;
import net.minecraft.util.Vec3;
import org.lwjgl.opengl.GL11;
import net.minecraftforge.common.MinecraftForge;
import net.minecraftforge.fml.common.eventhandler.SubscribeEvent;
import net.minecraftforge.client.event.RenderWorldLastEvent;

public class old2 {
    private final World world;
    private final MovementController movementController;
    private final EnvironmentScanner environmentScanner;

    // Thread-safe path reference
    private final AtomicReference<Path> currentPath = new AtomicReference<>();
    private volatile boolean isCalculating = false;
    private volatile boolean noPathFound = false;

    // Executor for background pathfinding
    private final ExecutorService pathfindingExecutor = Executors.newSingleThreadExecutor(r -> {
        Thread t = new Thread(r, "SAStarPathfinder");
        t.setDaemon(true);
        return t;
    });

    private Future<?> currentPathfindingTask;

    // Configuration constants
    private static final int MAX_ITERATIONS = 10000;
    private static final double HEURISTIC_WEIGHT = 1.2;
    private static final int RECALC_COOLDOWN_MS = 1000;
    private static final double NODE_REACH_DISTANCE = 1.5;
    private static final double GOAL_REACH_DISTANCE = 0.8;

    // Block movement costs
    private final Map<Block, Double> blockCosts = new HashMap<>();

    // Timing control
    private long lastRecalcTime = 0;
    private long lastPathfindRequest = 0;

    // Debug rendering
    public boolean debugEnabled = false;

    public old2(World world) {
        this.world = world;
        this.movementController = new MovementController();
        this.environmentScanner = new EnvironmentScanner(world);
        initializeBlockCosts();
        MinecraftForge.EVENT_BUS.register(this);
    }

    private void initializeBlockCosts() {
        blockCosts.put(Blocks.soul_sand, 2.0);
        blockCosts.put(Blocks.water, 1.8);
        blockCosts.put(Blocks.flowing_water, 1.8);
        blockCosts.put(Blocks.lava, 100.0);
        blockCosts.put(Blocks.flowing_lava, 100.0);
        blockCosts.put(Blocks.cactus, 50.0);
        blockCosts.put(Blocks.fire, 100.0);
        // Stairs are slightly more expensive but still preferred for vertical movement
        blockCosts.put(Blocks.stone_stairs, 1.1);
        blockCosts.put(Blocks.oak_stairs, 1.1);
        blockCosts.put(Blocks.spruce_stairs, 1.1);
        blockCosts.put(Blocks.birch_stairs, 1.1);
        blockCosts.put(Blocks.jungle_stairs, 1.1);
        blockCosts.put(Blocks.acacia_stairs, 1.1);
        blockCosts.put(Blocks.dark_oak_stairs, 1.1);
        blockCosts.put(Blocks.brick_stairs, 1.1);
        blockCosts.put(Blocks.stone_brick_stairs, 1.1);
        blockCosts.put(Blocks.nether_brick_stairs, 1.1);
        blockCosts.put(Blocks.sandstone_stairs, 1.1);
        blockCosts.put(Blocks.quartz_stairs, 1.1);
    }

    /**
     * Request pathfinding from start to goal asynchronously
     */
    public void findPathAsync(BlockPos start, BlockPos goal) {
        if (start.equals(goal)) {
            currentPath.set(null);
            noPathFound = false;
            return;
        }

        long currentTime = System.currentTimeMillis();
        if (currentTime - lastPathfindRequest < 100) {
            return; // Throttle requests
        }
        lastPathfindRequest = currentTime;

        // Cancel any existing pathfinding task
        if (currentPathfindingTask != null && !currentPathfindingTask.isDone()) {
            currentPathfindingTask.cancel(true);
        }

        isCalculating = true;
        currentPathfindingTask = pathfindingExecutor.submit(() -> {
            try {
                Path path = calculatePath(start, goal);
                if (!Thread.currentThread().isInterrupted()) {
                    currentPath.set(path);
                    noPathFound = (path == null);
                }
            } catch (Exception e) {
                if (debugEnabled) {
                    System.err.println("Pathfinding error: " + e.getMessage());
                }
                noPathFound = true;
            } finally {
                isCalculating = false;
            }
        });
    }

    /**
     * Core A* pathfinding algorithm
     */
    private Path calculatePath(BlockPos start, BlockPos goal) {
        if (Thread.currentThread().isInterrupted()) return null;

        PriorityQueue<PathNode> openSet = new PriorityQueue<>();
        Set<BlockPos> closedSet = new HashSet<>();
        Map<BlockPos, PathNode> allNodes = new HashMap<>();

        PathNode startNode = new PathNode(start, 0, heuristic(start, goal));
        openSet.offer(startNode);
        allNodes.put(start, startNode);

        int iterations = 0;

        while (!openSet.isEmpty() && iterations < MAX_ITERATIONS && !Thread.currentThread().isInterrupted()) {
            iterations++;

            PathNode current = openSet.poll();

            if (current.pos.equals(goal)) {
                return reconstructPath(current, start, goal);
            }

            closedSet.add(current.pos);

            for (BlockPos neighbor : getValidNeighbors(current.pos, goal)) {
                if (closedSet.contains(neighbor) || !environmentScanner.isChunkLoaded(neighbor)) {
                    continue;
                }

                double tentativeG = current.gCost + getMovementCost(current.pos, neighbor);
                PathNode neighborNode = allNodes.get(neighbor);

                if (neighborNode == null) {
                    neighborNode = new PathNode(neighbor, tentativeG, heuristic(neighbor, goal));
                    neighborNode.parent = current;
                    neighborNode.action = determineAction(current.pos, neighbor);
                    allNodes.put(neighbor, neighborNode);
                    openSet.offer(neighborNode);
                } else if (tentativeG < neighborNode.gCost) {
                    neighborNode.gCost = tentativeG;
                    neighborNode.fCost = tentativeG + neighborNode.hCost * HEURISTIC_WEIGHT;
                    neighborNode.parent = current;
                    neighborNode.action = determineAction(current.pos, neighbor);

                    // Re-add to open set if not already there
                    if (!openSet.contains(neighborNode)) {
                        openSet.offer(neighborNode);
                    }
                }
            }
        }

        if (debugEnabled) {
            System.out.println("Pathfinding completed after " + iterations + " iterations. No path found.");
        }

        return null; // No path found
    }

    private Path reconstructPath(PathNode goalNode, BlockPos start, BlockPos goal) {
        List<PathNode> pathNodes = new ArrayList<>();
        PathNode current = goalNode;

        while (current != null) {
            pathNodes.add(current);
            current = current.parent;
        }

        Collections.reverse(pathNodes);

        // Simplify path by removing unnecessary intermediate nodes
        List<PathNode> simplifiedPath = simplifyPath(pathNodes);

        return new Path(simplifiedPath, start, goal);
    }

    private List<PathNode> simplifyPath(List<PathNode> originalPath) {
        if (originalPath.size() <= 2) return originalPath;

        List<PathNode> simplified = new ArrayList<>();
        simplified.add(originalPath.get(0));

        for (int i = 1; i < originalPath.size() - 1; i++) {
            PathNode prev = originalPath.get(i - 1);
            PathNode current = originalPath.get(i);
            PathNode next = originalPath.get(i + 1);

            // Keep node if it's a significant direction change or action change
            if (!canSkipNode(prev, current, next)) {
                simplified.add(current);
            }
        }

        simplified.add(originalPath.get(originalPath.size() - 1));
        return simplified;
    }

    private boolean canSkipNode(PathNode prev, PathNode current, PathNode next) {
        // Don't skip if there's an action change
        if (current.action != ActionType.WALK || next.action != ActionType.WALK) {
            return false;
        }

        // Don't skip if we're on stairs - need precise movement
        if (environmentScanner.isStair(current.pos) || environmentScanner.isStair(next.pos)) {
            return false;
        }

        // Don't skip if there's a significant height change
        if (Math.abs(next.pos.getY() - prev.pos.getY()) > 1) {
            return false;
        }

        // Check if we can move directly from prev to next
        return environmentScanner.hasDirectPath(prev.pos, next.pos);
    }

    private List<BlockPos> getValidNeighbors(BlockPos pos, BlockPos goal) {
        List<BlockPos> neighbors = new ArrayList<>();

        // Prioritize directions toward goal
        int dx = Integer.signum(goal.getX() - pos.getX());
        int dz = Integer.signum(goal.getZ() - pos.getZ());
        int dy = Integer.signum(goal.getY() - pos.getY());

        // Movement directions in order of preference
        int[][] directions = {
                {dx, 0, dz},     // Diagonal toward goal
                {dx, 0, 0},      // X toward goal
                {0, 0, dz},      // Z toward goal
                {0, 1, 0},       // Up
                {0, -1, 0},      // Down
                {1, 0, 0}, {-1, 0, 0}, {0, 0, 1}, {0, 0, -1}, // Cardinals
                {1, 0, 1}, {1, 0, -1}, {-1, 0, 1}, {-1, 0, -1} // Diagonals
        };

        for (int[] dir : directions) {
            BlockPos neighbor = pos.add(dir[0], dir[1], dir[2]);

            if (isValidMove(pos, neighbor)) {
                neighbors.add(neighbor);
            }
        }

        // Add single block jumping neighbors
        addJumpingNeighbors(pos, neighbors);

        // Add staircase-specific neighbors
        addStaircaseNeighbors(pos, neighbors);

        // Add obstacle avoidance neighbors
        addObstacleAvoidanceNeighbors(pos, goal, neighbors);

        return neighbors;
    }

    private void addJumpingNeighbors(BlockPos pos, List<BlockPos> neighbors) {
        // Check each horizontal direction for single block obstacles
        for (EnumFacing facing : EnumFacing.HORIZONTALS) {
            BlockPos obstaclePos = pos.offset(facing);
            BlockPos landingPos = obstaclePos.offset(facing);

            // Check if there's a single block we can jump over
            if (canJumpOverSingleBlock(pos, obstaclePos, landingPos)) {
                neighbors.add(landingPos);

                // Also add the position on top of the obstacle if it's walkable
                BlockPos onTop = obstaclePos.up();
                if (isValidMove(pos, onTop)) {
                    neighbors.add(onTop);
                }
            }

            // Check diagonal jumping over single blocks
            for (EnumFacing diagonalFacing : EnumFacing.HORIZONTALS) {
                if (diagonalFacing == facing || diagonalFacing == facing.getOpposite()) continue;

                BlockPos diagonalObstacle = pos.offset(facing).offset(diagonalFacing);
                BlockPos diagonalLanding = diagonalObstacle.offset(facing);

                if (canJumpOverSingleBlock(pos, diagonalObstacle, diagonalLanding)) {
                    neighbors.add(diagonalLanding);
                }
            }
        }
    }

    private void addObstacleAvoidanceNeighbors(BlockPos pos, BlockPos goal, List<BlockPos> neighbors) {
        // Check if there are obstacles between current position and goal
        if (hasDangerousPathToGoal(pos, goal) || environmentScanner.hasObstacles(pos, goal)) {
            // Calculate direction to goal
            Vec3 toGoal = new Vec3(goal.getX() - pos.getX(), 0, goal.getZ() - pos.getZ()).normalize();
            
            // Create perpendicular vectors for detour options
            Vec3 perpendicular1 = new Vec3(-toGoal.zCoord, 0, toGoal.xCoord);
            Vec3 perpendicular2 = new Vec3(toGoal.zCoord, 0, -toGoal.xCoord);

            // Try positions at various distances perpendicular to the direct path
            for (int distance = 1; distance <= 4; distance++) { // Increased max distance
                // Add positions on both sides of the direct path
                addPerpendicularNeighbor(pos, perpendicular1, distance, neighbors);
                addPerpendicularNeighbor(pos, perpendicular2, distance, neighbors);
                
                // Also add positions slightly forward of the perpendicular points
                Vec3 forward1 = new Vec3(toGoal.xCoord * 0.5, 0, toGoal.zCoord * 0.5);
                Vec3 forward2 = new Vec3(toGoal.xCoord * 0.5, 0, toGoal.zCoord * 0.5);
                
                addPerpendicularNeighbor(pos.add(
                    (int)(perpendicular1.xCoord * distance + forward1.xCoord),
                    0,
                    (int)(perpendicular1.zCoord * distance + forward1.zCoord)
                ), neighbors);
                
                addPerpendicularNeighbor(pos.add(
                    (int)(perpendicular2.xCoord * distance + forward2.xCoord),
                    0,
                    (int)(perpendicular2.zCoord * distance + forward2.zCoord)
                ), neighbors);
            }
        }
    }

    private void addPerpendicularNeighbor(BlockPos pos, Vec3 direction, int distance, List<BlockPos> neighbors) {
        BlockPos candidate = pos.add(
            (int)(direction.xCoord * distance),
            0,
            (int)(direction.zCoord * distance)
        );

        if (isValidMove(pos, candidate) && !environmentScanner.isDangerous(candidate)) {
            neighbors.add(candidate);
        }
    }

    private void addPerpendicularNeighbor(BlockPos candidate, List<BlockPos> neighbors) {
        if (isValidMove(candidate, candidate) && !environmentScanner.isDangerous(candidate)) {
            neighbors.add(candidate);
        }
    }

    private boolean isValidMove(BlockPos from, BlockPos to) {
        if (!environmentScanner.isChunkLoaded(to)) return false;

        int dy = to.getY() - from.getY();

        // Allow falling into air or liquid or onto non-solid blocks below
        if (dy < 0) {
            // When falling, target block itself must not be solid, dangerous, and there must be solid/walkable ground below
            return !environmentScanner.isSolid(to) &&
                   !environmentScanner.isDangerous(to) &&
                   (environmentScanner.isSolid(to.down()) || environmentScanner.isWalkable(to.down()) || environmentScanner.isLiquid(to)); // Allow falling into liquid
        }

        // For horizontal or upward movement, check if the neighbor is walkable and not an obstacle
        if (dy == 0 || dy == 1) {
            // Can only move up 1 block at a time normally (stair steps handled separately)
            if (dy == 1 && !environmentScanner.isStair(to)) {
                 // When moving up 1, target block must be non-solid, have space above, AND have solid/walkable ground at the target level
                 if (environmentScanner.isSolid(to) || // Target block itself is solid
                     environmentScanner.isSolid(to.up()) || environmentScanner.isSolid(to.up(2)) || // Not enough head clearance
                     (!environmentScanner.isSolid(to.down()) && !environmentScanner.isWalkable(to.down()))) { // No solid/walkable ground directly below target
                      return false;
                 }
            }

            // Check if the neighbor is walkable and there's no obstacle between (considering player height)
            // Use hasObstacles for a more thorough check of the path volume
            if (!environmentScanner.isWalkable(to) || environmentScanner.hasObstacles(from, to)) {
                return false;
            }

            // For horizontal moves (dy == 0), ensure there is solid ground or a walkable block directly below the neighbor position.
            if (dy == 0) {
                 BlockPos blockBelowNeighbor = to.down();
                 if (!environmentScanner.isSolid(blockBelowNeighbor) && !environmentScanner.isWalkable(blockBelowNeighbor)) {
                      return false; // Prevent moving horizontally off an edge
                 }
            }

            return true; // Valid horizontal or upward neighbor
        }

        // Disallow moving up by more than 1 block in a single step normally (jumps handled by addJumpingNeighbors and isJumpMove)
        if (dy > 1) return false;

        return false; // Should not reach here for valid movements
    }

    private boolean canJumpOverSingleBlock(BlockPos start, BlockPos obstacle, BlockPos landing) {
        // Check if obstacle position has a solid block at player foot level and is only 1 block high
        if (!environmentScanner.isSolid(obstacle) || environmentScanner.isSolid(obstacle.up())) {
            return false;
        }

        // Check if we have enough space to jump over the obstacle (2 blocks high clearance from start)
        if (environmentScanner.isSolid(start.up()) || environmentScanner.isSolid(start.up(2))) {
            return false;
        }

        // Check if landing position is safe and walkable, and has enough space above it
        if (!isValidMove(obstacle.up(), landing.up()) || // Check validity at player foot level at landing (uses refined isValidMove)
            environmentScanner.isSolid(landing.up()) || environmentScanner.isSolid(landing.up(2))) { // Check head clearance at landing
            return false;
        }

        // Check if landing position isn't too high or low compared to the start (allow 1 block up/down for jumps)
        int heightDiff = Math.abs(landing.getY() - start.getY());
        if (heightDiff > 1) { // Allow height difference of 1 for jumps
            return false;
        }

        // Check if there is no solid block directly above the obstacle at jump height
         if (environmentScanner.isSolid(obstacle.up(2))) {
              return false;
         }

        return true; // Valid single block jump
    }

    private boolean hasDangerousPathToGoal(BlockPos start, BlockPos goal) {
        Vec3 startVec = new Vec3(start.getX() + 0.5, start.getY() + 0.5, start.getZ() + 0.5);
        Vec3 goalVec = new Vec3(goal.getX() + 0.5, goal.getY() + 0.5, goal.getZ() + 0.5);

        // Sample points along the direct path
        int samples = (int) Math.max(Math.abs(goal.getX() - start.getX()), Math.abs(goal.getZ() - start.getZ()));
        samples = Math.max(samples, 1);

        for (int i = 0; i <= samples; i++) {
            double t = (double) i / samples;
            Vec3 samplePos = startVec.addVector(
                    (goalVec.xCoord - startVec.xCoord) * t,
                    (goalVec.yCoord - startVec.yCoord) * t,
                    (goalVec.zCoord - startVec.zCoord) * t
            );

            BlockPos checkPos = new BlockPos(samplePos);
            if (environmentScanner.isDangerous(checkPos) ||
                environmentScanner.isDangerous(checkPos.down()) || // Check block below
                environmentScanner.isDangerous(checkPos.up())) { // Check block above (head level)
                return true;
            }
        }

        return false;
    }

    private double getMovementCost(BlockPos from, BlockPos to) {
        double distance = Math.sqrt(from.distanceSq(to));
        double cost = distance;

        // Block-specific costs
        Block block = world.getBlockState(to).getBlock();
        cost *= blockCosts.getOrDefault(block, 1.0);

        // Special stair handling
        if (environmentScanner.isStair(to) && environmentScanner.isProperStairMovement(from, to)) {
            cost *= 0.9; // Slightly reduced cost for proper stair movement
        }

        // Height change penalties
        int dy = to.getY() - from.getY();
        if (dy > 0) {
            if (environmentScanner.isStair(to) || environmentScanner.isStair(from)) {
                cost += dy * 0.3; // Lower penalty for going up stairs
            } else {
                cost += dy * 0.8; // Higher penalty for jumping/climbing other blocks
            }
        } else if (dy < 0) {
            cost += Math.abs(dy) * 0.3; // Small cost for falling
        }

        // Diagonal movement penalty
        if (Math.abs(to.getX() - from.getX()) > 0 && Math.abs(to.getZ() - from.getZ()) > 0) {
            cost *= 1.414; // Diagonal distance is sqrt(2) times horizontal/vertical
        }

        // Jumping penalty (if this is a jump move defined by isJumpMove)
        if (isJumpMove(from, to)) {
            cost += 1.0; // Add jumping cost (can be tuned)
        }

        // Proximity to danger penalty (encourage staying away from lava, etc.)
        double dangerProximityPenalty = calculateDangerProximityPenalty(to);
        cost += dangerProximityPenalty * 10.0; // Scale danger penalty

        // Environmental penalties
        if (environmentScanner.isLiquid(to)) cost *= 2.0; // Penalty for water/lava
        if (environmentScanner.isDangerous(to)) cost *= 10.0; // Penalty for fire/lava

        return cost;
    }

    private boolean isJumpMove(BlockPos from, BlockPos to) {
        // Check if we're skipping over a solid block horizontally
        int dx = to.getX() - from.getX();
        int dz = to.getZ() - from.getZ();

        if (Math.abs(dx) == 2 && dz == 0) {
            BlockPos middle = from.add(Integer.signum(dx), 0, 0);
            if (environmentScanner.isSolid(middle) && !environmentScanner.isSolid(middle.up())) return true;
        }
        if (Math.abs(dz) == 2 && dx == 0) {
             BlockPos middle = from.add(0, 0, Integer.signum(dz));
             if (environmentScanner.isSolid(middle) && !environmentScanner.isSolid(middle.up())) return true;
        }
         // Check for diagonal jump over a corner block
        if (Math.abs(dx) == 1 && Math.abs(dz) == 1) {
             BlockPos corner1 = from.add(dx, 0, 0);
             BlockPos corner2 = from.add(0, 0, dz);
             // If both adjacent blocks are solid, it might be a corner jump
             if (environmentScanner.isSolid(corner1) && environmentScanner.isSolid(corner2)) {
                 // Check if there's space over the corner
                 if (!environmentScanner.isSolid(corner1.up()) && !environmentScanner.isSolid(corner2.up())){
                     return true;
                 }
             }
        }

        // Check if moving up by more than 1 block in a single step (would require a jump)
        if (to.getY() - from.getY() > 1) {
             return true;
        }

        return false;
    }

    private double calculateDangerProximityPenalty(BlockPos pos) {
        double penalty = 0.0;

        // Check surrounding blocks for danger in a wider radius
        for (int x = -3; x <= 3; x++) {
            for (int z = -3; z <= 3; z++) {
                for (int y = -2; y <= 2; y++) { // Check vertically as well
                    BlockPos checkPos = pos.add(x, y, z);
                    if (environmentScanner.isDangerous(checkPos)) {
                        double distance = Math.sqrt(x*x + y*y + z*z);
                        // Apply a penalty that decreases with distance, significant when close
                        penalty += Math.max(0, 3.0 - distance); // Increase radius and base penalty
                    }
                }
            }
        }

        return penalty;
    }

    private ActionType determineAction(BlockPos from, BlockPos to) {
        int dy = to.getY() - from.getY();

        // Special handling for stairs
        if (environmentScanner.isStair(to) || environmentScanner.isStair(from)) {
            if (dy > 0) return ActionType.STAIR_UP;
            if (dy < 0) return ActionType.STAIR_DOWN;
            return ActionType.STAIR_WALK;
        }

        // Check if this is a jump move (over obstacles or significant vertical change)
        if (isJumpMove(from, to)) {
            return ActionType.JUMP;
        }

        // Refine upward movement action determination: Only classify as JUMP if isJumpMove is true.
        // If moving up and not a stair and not a recognized jump, treat as a walk step-up
        if (dy > 0) {
            // Given that isValidMove already passed, we assume this is a reachable upward step.
            // Classify as WALK for a simple step-up movement.
            return ActionType.WALK; 
        }

        if (dy < 0) return ActionType.FALL;
        return ActionType.WALK;
    }

    private double heuristic(BlockPos a, BlockPos b) {
        // Manhattan distance for better performance
        return Math.abs(a.getX() - b.getX()) + 
               Math.abs(a.getY() - b.getY()) + 
               Math.abs(a.getZ() - b.getZ());
    }

    /**
     * Called every client tick (if a path is active). Moves `entity` along the path.
     */
    public void followPath(EntityLivingBase entity) {
        Path currentPath = this.currentPath.get();
        if (currentPath == null || currentPath.isComplete()) {
            return;
        }
        movementController.executeMovement(entity, currentPath);
        if (debugEnabled) {
            renderPath();
        }
    }

    /**
     * If stuck for more than ~2 seconds (40 ticks), recalc a new path to the same goal.
     */
    public void recalculateIfNeeded(BlockPos currentPos, BlockPos goalPos) {
        Path currentPath = this.currentPath.get();
        if (currentPath != null && currentPath.isStuck(currentPos)) {
            long currentTime = System.currentTimeMillis();
            // Use a separate cooldown for recalculation requests from MovementController
            if (currentTime - lastRecalcTime > RECALC_COOLDOWN_MS) {
                if (debugEnabled) {
                    System.out.println("Pathfinder: Recalculating path due to being stuck.");
                }
                findPathAsync(currentPos, goalPos);
                lastRecalcTime = currentTime;
            }
        }
    }

    @SubscribeEvent
    public void onRenderWorldLast(RenderWorldLastEvent event) {
        if (debugEnabled && currentPath.get() != null) {
            renderPath();
        }
    }

    private void renderPath() {
        Path currentPath = this.currentPath.get();
        if (currentPath == null || currentPath.nodes.isEmpty()) return;

        Minecraft mc = Minecraft.getMinecraft();
        if (mc.getRenderManager() == null || mc.thePlayer == null) return; // Ensure render manager and player are available

        double renderPosX = mc.getRenderManager().viewerPosX;
        double renderPosY = mc.getRenderManager().viewerPosY;
        double renderPosZ = mc.getRenderManager().viewerPosZ;

        GL11.glPushMatrix();
        GL11.glDisable(GL11.GL_TEXTURE_2D);
        GL11.glDisable(GL11.GL_DEPTH_TEST);
        GL11.glEnable(GL11.GL_BLEND);
        GL11.glBlendFunc(GL11.GL_SRC_ALPHA, GL11.GL_ONE_MINUS_SRC_ALPHA);
        GL11.glLineWidth(2.0f);

        // Draw path lines (single color for now)
        GL11.glBegin(GL11.GL_LINE_STRIP);
        GL11.glColor4f(0.0f, 1.0f, 0.0f, 0.7f); // Green with some transparency

        for (PathNode node : currentPath.nodes) {
            BlockPos pos = node.pos;
            GL11.glVertex3d(pos.getX() + 0.5 - renderPosX, pos.getY() + 0.5 - renderPosY, pos.getZ() + 0.5 - renderPosZ);
        }
        GL11.glEnd();

        // Draw current node marker
        if (currentPath.currentIndex < currentPath.nodes.size()) {
            PathNode currentNode = currentPath.nodes.get(currentPath.currentIndex);
            BlockPos pos = currentNode.pos;
            GL11.glColor4f(1.0f, 1.0f, 0.0f, 1.0f); // Yellow for current node
            GL11.glPointSize(8.0f);
            GL11.glBegin(GL11.GL_POINTS);
            GL11.glVertex3d(pos.getX() + 0.5 - renderPosX, pos.getY() + 0.5 - renderPosY, pos.getZ() + 0.5 - renderPosZ);
            GL11.glEnd();
        }

        GL11.glEnable(GL11.GL_TEXTURE_2D);
        GL11.glEnable(GL11.GL_DEPTH_TEST);
        GL11.glDisable(GL11.GL_BLEND);
        GL11.glPopMatrix();
    }

    /** Cancel any existing path immediately. */
    public void cancelPath() {
        if (currentPathfindingTask != null && !currentPathfindingTask.isDone()) {
            currentPathfindingTask.cancel(true);
        }
        currentPath.set(null);
        noPathFound = false;
        isCalculating = false;
    }

    /** Returns true if a path is currently being calculated. */
    public boolean isCalculating() {
        return isCalculating;
    }

    /** Returns true if the last pathfinding attempt failed to find a path. */
    public boolean hasNoPath() {
        return noPathFound;
    }

    /**
     * Returns true if either:
     *  - A valid path was found and you have walked it all the way through, OR
     *  - No path was found in the last A* attempt (so there's nothing to walk).
     */
    public boolean isPathComplete() {
        Path path = currentPath.get();
        if (path == null) {
            return !isCalculating;
        }
        return path.isComplete();
    }

    /** Adjust the movement cost for a specific block type. */
    public void setBlockCost(Block block, double cost) {
        blockCosts.put(block, cost);
    }

    /** Enable/disable console debug prints. */
    public void setDebugEnabled(boolean enabled) {
        debugEnabled = enabled;
    }

    /** Shut down the A* executor thread (call on mod unload). */
    public void shutdown() {
        pathfindingExecutor.shutdownNow(); // Attempt to stop currently executing tasks
    }

    private static class PathNode implements Comparable<PathNode> {
        final BlockPos pos;
        PathNode parent;
        double gCost;
        final double hCost;
        double fCost;
        ActionType action = ActionType.WALK; // Default action

        PathNode(BlockPos pos, double gCost, double hCost) {
            this(pos, null, gCost, hCost);
        }

        PathNode(BlockPos pos, PathNode parent, double gCost, double hCost) {
            this.pos = pos;
            this.parent = parent;
            this.gCost = gCost;
            this.hCost = hCost;
            this.fCost = gCost + hCost; // Initial fCost
        }

        // For priority queue
        @Override
        public int compareTo(PathNode other) {
            return Double.compare(this.fCost, other.fCost);
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (obj == null || getClass() != obj.getClass()) return false;
            PathNode pathNode = (PathNode) obj;
            return pos.equals(pathNode.pos);
        }

        @Override
        public int hashCode() {
            return pos.hashCode();
        }
    }

    public static class Path {
        final List<PathNode> nodes;
        final BlockPos start;
        final BlockPos goal;
        int currentIndex = 0;
        private boolean completed = false;
        private BlockPos lastPosition;
        private int stuckCounter = 0;

        Path(List<PathNode> nodes, BlockPos start, BlockPos goal) {
            this.nodes = nodes;
            this.start = start;
            this.goal = goal;
            if (!nodes.isEmpty()) {
                this.lastPosition = nodes.get(0).pos;
            }
        }

        public PathNode getCurrentNode() {
            if (currentIndex >= nodes.size()) return null;
            return nodes.get(currentIndex);
        }

        public void advance() {
            if (currentIndex < nodes.size()) {
                 lastPosition = nodes.get(currentIndex).pos; // Update last position before advancing
                currentIndex++;
                if (currentIndex >= nodes.size()) {
                    completed = true;
                }
            }
        }

        public boolean isComplete() {
            // Path is complete if all nodes visited or if explicitly marked complete
            return completed || (nodes.isEmpty() && currentIndex >= 0); // Handle empty path case
        }

        public boolean isStuck(BlockPos currentPos) {
             if (lastPosition == null || currentPos == null) return false;

             if (currentPos.equals(lastPosition)) {
                 stuckCounter++;
             } else {
                 stuckCounter = 0;
                 lastPosition = currentPos;
             }

             // Consider stuck if at the same block for a number of ticks (e.g., 20 ticks = 1 second)
             return stuckCounter >= 20;
        }

        public boolean hasReachedGoal(EntityLivingBase entity) {
            if (goal == null) return false;
            // Check if the entity is within the block of the goal position
            return new BlockPos(entity).equals(goal);
        }
    }

    private class MovementController {
        private final Minecraft mc = Minecraft.getMinecraft();
        private int jumpCooldown = 0;
        private boolean wasOnGround = true;
        private BlockPos lastTargetPos = null;
        private int precisionMoveCounter = 0;

        void executeMovement(EntityLivingBase entity, Path path) {
            PathNode currentNode = path.getCurrentNode();
            if (currentNode == null) {
                releaseAllKeysSmoothly(); // Release keys when path is complete
                return;
            }

            // Update jump cooldown
            if (jumpCooldown > 0) {
                jumpCooldown--;
            }

            // Check if we've reached the current node
            double distanceToNode = entity.getDistanceSqToCenter(currentNode.pos);
            boolean reachedNode = false;

            // Different reach distances based on action type and vertical movement
            double reachDistance = NODE_REACH_DISTANCE;
            // Use a smaller reach distance for vertical movements and stairs for more precision
            if (currentNode.action == ActionType.JUMP || currentNode.action == ActionType.FALL ||
                currentNode.action == ActionType.STAIR_UP || currentNode.action == ActionType.STAIR_DOWN ||
                currentNode.action == ActionType.STAIR_WALK || Math.abs(currentNode.pos.getY() - entity.posY) > 0.1) { // Also check if there's vertical difference
                reachDistance = 0.8; // More precise for verticality and stairs
            }

            // Check if entity is close enough to the target node position (considering the center of the block)
            if (distanceToNode < reachDistance * reachDistance) {
                reachedNode = true;
            }

            // Also check if we're close enough and potentially moving away or stuck (prevent overshooting and handle getting stuck)
             if (!reachedNode && lastTargetPos != null && lastTargetPos.equals(currentNode.pos)) {
                 precisionMoveCounter++;
                 // If stuck near the node for a few ticks, consider it reached
                 if (precisionMoveCounter > 5 && distanceToNode < (reachDistance * 1.5) * (reachDistance * 1.5)) { // Check slightly further if stuck near
                     reachedNode = true;
                 }
             } else {
                 precisionMoveCounter = 0;
                 lastTargetPos = currentNode.pos; // Update last target position
             }

            if (reachedNode) {
                path.advance();
                currentNode = path.getCurrentNode();
                if (currentNode == null) {
                    releaseAllKeysSmoothly(); // Release keys when path is complete after advancing
                    return;
                }
                precisionMoveCounter = 0; // Reset counter on advancing
                lastTargetPos = currentNode.pos; // Update last target position for the new node
            }

            // Calculate target position for rotation and movement
            Vec3 targetPos = new Vec3(
                    currentNode.pos.getX() + 0.5,
                    // Aim slightly above the block for walking/horizontal to account for player height
                    // For vertical moves, aim closer to the center of the target block vertically
                    currentNode.action == ActionType.WALK || currentNode.action == ActionType.STAIR_WALK ? currentNode.pos.getY() + 0.1 : currentNode.pos.getY() + 0.5,
                    currentNode.pos.getZ() + 0.5
            );
             if (currentNode.action == ActionType.JUMP || currentNode.action == ActionType.STAIR_UP) {
                 // For jumping/stair up, aim towards the top of the target block
                 targetPos = new Vec3(currentNode.pos.getX() + 0.5, currentNode.pos.getY() + 1.0, currentNode.pos.getZ() + 0.5); // Aim higher for jump/stair up
             } else if (currentNode.action == ActionType.FALL) {
                 // When falling, aim towards the bottom of the target block
                 targetPos = new Vec3(currentNode.pos.getX() + 0.5, currentNode.pos.getY(), currentNode.pos.getZ() + 0.5); // Aim lower for falling
             }

            // Handle rotation
            PathFindRotation.RotationUtils.rotateToPosition(entity, targetPos);

            // Handle movement based on action type
            executeActionMovement(entity, currentNode, targetPos);

            wasOnGround = entity.onGround;
        }

        private void executeActionMovement(EntityLivingBase entity, PathNode node, Vec3 targetPos) {
            boolean shouldMoveForward = PathFindRotation.RotationUtils.isFacingPosition(entity, targetPos, 25.0f);

            // Clear all movement keys first, except potentially jump if we are in a jump action
             if (node.action != ActionType.JUMP) {
                releaseAllKeysSmoothly();
             } else {
                 // Only release non-jump keys during a jump action
                 setKey(mc.gameSettings.keyBindForward, false);
                 setKey(mc.gameSettings.keyBindBack, false);
                 setKey(mc.gameSettings.keyBindLeft, false);
                 setKey(mc.gameSettings.keyBindRight, false);
                 setKey(mc.gameSettings.keyBindSprint, false);
             }

            switch (node.action) {
                case WALK:
                    handleWalkMovement(entity, targetPos, shouldMoveForward);
                    break;

                case JUMP:
                    handleJumpMovement(entity, node, targetPos, shouldMoveForward);
                    break;

                case STAIR_WALK:
                case STAIR_UP:
                case STAIR_DOWN:
                    handleStairMovement(entity, node, targetPos, shouldMoveForward);
                    break;

                case FALL:
                    handleFallMovement(entity, targetPos, shouldMoveForward);
                    break;
            }
        }

        private void handleWalkMovement(EntityLivingBase entity, Vec3 targetPos, boolean shouldMoveForward) {
            setKey(mc.gameSettings.keyBindForward, shouldMoveForward);

            // Handle automatic jumping for small obstacles during walk
            // This should be less aggressive now that pathfinder generates jump nodes
            if (shouldMoveForward && entity.onGround && jumpCooldown == 0) {
                BlockPos entityPos = new BlockPos(entity);
                BlockPos forward = entityPos.offset(entity.getHorizontalFacing());

                // Check for a solid block in front at current level that is *not* a stair or similar easily stepped block
                if (environmentScanner.isSolid(forward) && !environmentScanner.isStair(forward) &&
                    !environmentScanner.isSolid(forward.up())) { // Ensure obstacle is only 1 block high
                     // Check if there's space above the obstacle and at the landing spot
                     BlockPos landingPos = forward.offset(entity.getHorizontalFacing());
                     if (!environmentScanner.isSolid(forward.up()) && !environmentScanner.isSolid(forward.up(2)) &&
                         !environmentScanner.isSolid(landingPos.up()) && !environmentScanner.isSolid(landingPos.up(2)) ){
                          setKey(mc.gameSettings.keyBindJump, true);
                          jumpCooldown = 8; // Short cooldown for auto-jump
                     }
                }
            }

            // Handle sprinting
            boolean shouldSprint = shouldMoveForward && entity.onGround && !entity.isInWater();
            setKey(mc.gameSettings.keyBindSprint, shouldSprint);
        }

        /**
         * Handle movement for jump actions (over obstacles or significant height changes)
         */
        private void handleJumpMovement(EntityLivingBase entity, PathNode node, Vec3 targetPos, boolean shouldMoveForward) {
            setKey(mc.gameSettings.keyBindForward, shouldMoveForward); // Keep moving forward while attempting jump

            // Trigger jump if on the ground and facing the right direction, and cooldown is ready
            if (entity.onGround && shouldMoveForward && jumpCooldown == 0) {
                 BlockPos entityPos = new BlockPos(entity);
                 BlockPos targetBlock = node.pos;

                 // If the action type is specifically JUMP or the target is significantly higher, trigger jump
                 // Added check to ensure the target block isn't just air or a non-supportive block where a jump would be invalid
                 if ((node.action == ActionType.JUMP || targetBlock.getY() > entityPos.getY()) && environmentScanner.hasSolidGround(targetBlock.down())) { // Ensure solid ground at target level
                     setKey(mc.gameSettings.keyBindJump, true);
                     jumpCooldown = 20; // Longer cooldown for planned jumps to allow jump animation
                 }
            }

            // Don't sprint when executing a planned jump action
            setKey(mc.gameSettings.keyBindSprint, false);
        }

        private void handleStairMovement(EntityLivingBase entity, PathNode node, Vec3 targetPos, boolean shouldMoveForward) {
            setKey(mc.gameSettings.keyBindForward, shouldMoveForward);

            // For stairs, we generally don't need to jump unless stepping up the first stair block
            if (shouldMoveForward && entity.onGround && jumpCooldown == 0) {
                BlockPos entityPos = new BlockPos(entity);
                BlockPos nextBlock = node.pos;

                // If moving to a stair node that is one block higher than current position, trigger a small jump/step
                if (environmentScanner.isStair(nextBlock) && nextBlock.getY() > entityPos.getY()) {
                     setKey(mc.gameSettings.keyBindJump, true);
                     jumpCooldown = 5; // Short cooldown for stair step
                }
                 // Removed the stuck check based on motion; relying on path recalculation for getting stuck
            }

            // Moderate sprinting on stairs - slower for precision
            boolean shouldSprint = shouldMoveForward && entity.onGround && !entity.isInWater() &&
                    node.action == ActionType.STAIR_WALK;
            setKey(mc.gameSettings.keyBindSprint, shouldSprint);
        }

        private void handleFallMovement(EntityLivingBase entity, Vec3 targetPos, boolean shouldMoveForward) {
            setKey(mc.gameSettings.keyBindForward, shouldMoveForward); // Continue moving forward while falling if applicable
            // No jumping when falling, let gravity do the work
            setKey(mc.gameSettings.keyBindSprint, false);
        }

        private void setKey(KeyBinding key, boolean pressed) {
            KeyBinding.setKeyBindState(key.getKeyCode(), pressed);
        }

        private void releaseAllKeysSmoothly() {
            setKey(mc.gameSettings.keyBindForward, false);
            setKey(mc.gameSettings.keyBindBack, false);
            setKey(mc.gameSettings.keyBindLeft, false);
            setKey(mc.gameSettings.keyBindRight, false);
            // Keep jump state, let the jump logic manage it
            setKey(mc.gameSettings.keyBindSprint, false);
        }
    }

    private class EnvironmentScanner {
        private final World world;

        EnvironmentScanner(World world) {
            this.world = world;
        }

        boolean isWalkable(BlockPos pos) {
            // Check if the position itself is passable (not a full solid block)
            if (isSolid(pos)) return false;

            // Check if there's solid ground below (or we're on a stair/slab/walkable block)
            BlockPos below = pos.down();
            if (!isSolid(below) && !isWalkable(below)) return false; // Needs solid or walkable block below

            // Check head clearance (the two blocks above the walkable block should not be solid)
            if (isSolid(pos.up()) || isSolid(pos.up(2))) return false; // Needs 2 blocks of air above

            Block block = world.getBlockState(pos).getBlock();
            // Consider blocks walkable if they are not full cubes or are stairs/slabs/ladders/vines etc.
            // This is a basic check, might need refinement.
            return !block.isNormalCube() || block instanceof BlockStairs || block instanceof BlockSlab ||
                   block instanceof BlockLadder || block instanceof BlockVine || block instanceof BlockSnow; // Added more walkable block types
        }

        boolean isSolid(BlockPos pos) {
            if (world == null || !isChunkLoaded(pos)) return true; // Assume solid if chunk not loaded
            IBlockState state = world.getBlockState(pos);
            Block block = state.getBlock();
            Material mat = state.getBlock().getMaterial();
            // A block is solid if it's a full normal cube and not a liquid or air
             return mat.isSolid() && block.isFullCube() && !(block instanceof BlockLiquid) && mat != Material.air;
        }

        boolean hasSolidGround(BlockPos pos) {
            return isSolid(pos);
        }

        boolean isStair(BlockPos pos) {
            if (world == null || !isChunkLoaded(pos)) return false;
            return world.getBlockState(pos).getBlock() instanceof BlockStairs;
        }

        /**
         * Placeholder for checking if a move is a proper stair movement based on stair orientation.
         * Needs detailed implementation.
         */
        boolean isProperStairMovement(BlockPos from, BlockPos to) {
            // Implement logic to check if the move from 'from' to 'to' is consistent
            // with navigating the stair block at 'to' (or 'from').
            return isStair(to) || isStair(from); // Basic check for now
        }

        public boolean hasDirectPath(BlockPos from, BlockPos to) {
            // Check if there's a clear line of sight between the points
            if (!hasLineOfSight(from, to)) {
                return false;
            }

            // Check if the height difference is manageable
            int heightDiff = Math.abs(to.getY() - from.getY());
            if (heightDiff > 1) {
                return false;
            }

            // Check if both positions are walkable
            if (!isWalkable(from) || !isWalkable(to)) {
                return false;
            }

            // Check for obstacles between the points
            Vec3 start = new Vec3(from.getX() + 0.5, from.getY() + 1.0, from.getZ() + 0.5);
            Vec3 end = new Vec3(to.getX() + 0.5, to.getY() + 1.0, to.getZ() + 0.5);
            
            // Check multiple points along the path
            for (double t = 0.0; t <= 1.0; t += 0.25) {
                double x = start.xCoord + (end.xCoord - start.xCoord) * t;
                double y = start.yCoord + (end.yCoord - start.yCoord) * t;
                double z = start.zCoord + (end.zCoord - start.zCoord) * t;
                
                BlockPos checkPos = new BlockPos(x, y, z);
                if (isDangerous(checkPos)) {
                    return false;
                }
                
                // Check for solid blocks that might block movement
                Block block = world.getBlockState(checkPos).getBlock();
                if (block.isNormalCube() && !(block instanceof BlockStairs) && !(block instanceof BlockSlab)) {
                    return false;
                }
            }

            return true;
        }

        public boolean hasLineOfSight(BlockPos from, BlockPos to) {
            Vec3 start = new Vec3(from.getX() + 0.5, from.getY() + 1.0, from.getZ() + 0.5);
            Vec3 end = new Vec3(to.getX() + 0.5, to.getY() + 1.0, to.getZ() + 0.5);
            
            // Check if there are any solid blocks between the points
            MovingObjectPosition mop = world.rayTraceBlocks(start, end, false, true, false);
            return mop == null || mop.typeOfHit == MovingObjectPosition.MovingObjectType.MISS;
        }

         boolean hasObstacles(BlockPos from, BlockPos to) {
            if (world == null || !isChunkLoaded(from) || !isChunkLoaded(to)) return true;
            
            // Check for obstacles using a ray trace and expanding the check around the line (player bounding box)
            double playerWidth = 0.6; // Increased from 0.3 to better detect walls
            double playerHeight = 1.8;

            double startX = from.getX() + 0.5;
            double startY = from.getY();
            double startZ = from.getZ() + 0.5;

            double endX = to.getX() + 0.5;
            double endY = to.getY();
            double endZ = to.getZ() + 0.5;

            Vec3 startVec = new Vec3(startX, startY, startZ);
            Vec3 endVec = new Vec3(endX, endY, endZ);

            // Calculate the number of steps based on the distance
            int steps = (int) Math.max(startVec.distanceTo(endVec) * 12, 4); // Increased granularity and minimum steps

            for (int i = 0; i <= steps; i++) {
                double t = (double) i / steps;
                double currentX = startVec.xCoord + (endVec.xCoord - startVec.xCoord) * t;
                double currentY = startVec.yCoord + (endVec.yCoord - startVec.yCoord) * t;
                double currentZ = startVec.zCoord + (endVec.zCoord - startVec.zCoord) * t;

                // Check a wider area around the path
                for (double xOffset = -playerWidth; xOffset <= playerWidth; xOffset += 0.3) {
                    for (double zOffset = -playerWidth; zOffset <= playerWidth; zOffset += 0.3) {
                        BlockPos checkPos = new BlockPos(
                            currentX + xOffset,
                            currentY,
                            currentZ + zOffset
                        );

                        // Check vertical space for the player
                        for (int y = 0; y <= playerHeight; y++) {
                            BlockPos verticalCheck = checkPos.up(y);
                            if (!isChunkLoaded(verticalCheck)) return true;

                            Block block = world.getBlockState(verticalCheck).getBlock();
                            IBlockState state = world.getBlockState(verticalCheck);

                            // More comprehensive obstacle check
                            if (block.isNormalCube() && !(block instanceof BlockStairs) && 
                                !(block instanceof BlockSlab) && !(block instanceof BlockLadder) && 
                                !(block instanceof BlockVine) && !(block instanceof BlockBarrier) &&
                                !(block instanceof BlockFence) && !(block instanceof BlockWall)) {
                                
                                // Check if the block is actually solid and not just a visual block
                                if (state.getBlock().getMaterial().isSolid() && 
                                    state.getBlock().isFullCube() && 
                                    !state.getBlock().getMaterial().isLiquid()) {
                                    return true;
                                }
                            }
                        }
                    }
                }
            }

            return false;
        }

        boolean canReach(BlockPos from, BlockPos to) {
            // Basic reachability check (within a certain distance and not through solid walls)
            if (from.distanceSq(to) > 25) return false; // Don't check if too far

            // Check for obstacles using hasObstacles (volumetric check)
            if (hasObstacles(from, to)) return false;

            // Check if the target block is walkable or can be moved into
            if (!isWalkable(to) && !isLiquid(to) && !isDangerous(to)) {
                 return false; // Cannot reach non-walkable, non-liquid, non-dangerous blocks
            }

            return true;
        }

        boolean needsJump(BlockPos from, BlockPos to) {
            // Determine if a jump is needed to move from 'from' to 'to'
            int dy = to.getY() - from.getY();

            // If moving up 1 block and not on stairs, usually needs a jump
            if (dy == 1 && !isStair(to)) {
                return true;
            }

            // If there's a solid block in between horizontally, needs a jump (handled by isJumpMove)
            // This method might be redundant if isJumpMove is used directly.

            return false;
        }

        boolean isChunkLoaded(BlockPos pos) {
            if (world == null || world.getChunkProvider() == null) return false;
            return world.getChunkProvider().chunkExists(pos.getX() >> 4, pos.getZ() >> 4);
        }

        boolean isDangerous(BlockPos pos) {
             if (world == null || !isChunkLoaded(pos)) return false;
            Block block = world.getBlockState(pos).getBlock();
            Material mat = block.getMaterial();
            return mat == Material.lava || mat == Material.fire || block instanceof BlockCactus;
        }

        boolean isLiquid(BlockPos pos) {
             if (world == null || !isChunkLoaded(pos)) return false;
            return world.getBlockState(pos).getBlock().getMaterial().isLiquid();
        }
    }

    private enum ActionType {
        WALK, JUMP, FALL, STAIR_WALK, STAIR_UP, STAIR_DOWN
    }

    private void addStaircaseNeighbors(BlockPos pos, List<BlockPos> neighbors) {
        // Check for stair traversal options
        for (EnumFacing facing : EnumFacing.HORIZONTALS) {
            BlockPos stairPos = pos.offset(facing);

            if (environmentScanner.isStair(stairPos)) {
                // Can walk on top of stairs
                BlockPos stairTop = stairPos.up();
                if (isValidMove(pos, stairTop)) {
                    neighbors.add(stairTop);
                }

                // Can walk under stairs if there's space
                if (isValidMove(pos, stairPos)) {
                    neighbors.add(stairPos);
                }
            }

            // Check for stair stepping (going up stairs naturally)
            BlockPos above = pos.up();
            BlockPos stairStep = above.offset(facing);
            if (environmentScanner.isStair(stairStep) && isValidMove(pos, stairStep)) {
                neighbors.add(stairStep);
            }
        }
    }
}