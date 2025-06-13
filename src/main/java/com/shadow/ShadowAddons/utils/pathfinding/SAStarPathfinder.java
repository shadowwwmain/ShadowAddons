package com.shadow.ShadowAddons.utils.pathfinding;

import net.minecraft.block.*;
import net.minecraft.block.state.IBlockState;
import net.minecraft.client.Minecraft;
import net.minecraft.client.renderer.GlStateManager;
import net.minecraft.entity.player.EntityPlayer;
import net.minecraft.init.Blocks;
import net.minecraft.util.AxisAlignedBB;
import net.minecraft.util.BlockPos;
import net.minecraft.world.World;
import net.minecraftforge.client.event.RenderWorldLastEvent;
import net.minecraftforge.common.MinecraftForge;
import net.minecraftforge.fml.common.eventhandler.SubscribeEvent;
import org.lwjgl.opengl.GL11;

import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

import static com.shadow.ShadowAddons.utils.pathfinding.SAStarPathfinder.PathNode.*;

public class SAStarPathfinder {

    //create updateMovementCapabilities method to have speedMultiplier and Jump boost level take them as arguments
    public static void updateMovementCapabilities(double speedMultiplier, double jumpBoostMultiplier) {
        Config.SPEED_MULTIPLIER = speedMultiplier;
        Config.JUMP_BOOST_MULTIPLIER = jumpBoostMultiplier;
        Config.MAX_JUMP_HEIGHT = (int) Math.ceil(1 * jumpBoostMultiplier); // Adjust max jump height based on multiplier
    }

    // Configuration
    public static class Config {
        public static int MAX_SEARCH_RADIUS = 150;
        public static int MAX_SEARCH_DEPTH = 1000;
        public static double VERTICAL_PENALTY = 1.5;  // Slightly prefer level paths
        public static int MAX_Y_DIFFERENCE = 50;  // Maximum Y difference to consider
        public static boolean AVOID_FENCES = true;
        public static boolean SMART_STAIR_CLIMBING = true;
        public static boolean ALLOW_CLIMBING = true;
        public static boolean ALLOW_SWIMMING = true;
        public static boolean ALLOW_UNSAFE_DROPS = true;  // Allow drops that might damage
        public static double OBSTACLE_PENALTY = 10.0;
        public static double FENCE_PENALTY = 20.0;
        public static double MAX_DROP_HEIGHT = 100;
        public static boolean DEBUG_ENABLED = false;
        public static boolean SHOW_PATH = true;
        public static boolean SHOW_NODES = false;
        public static boolean SHOW_COSTS = false;
        public static boolean SHOW_SEARCH_AREA = false;
        public static boolean SHOW_METRICS = true;
        public static HeuristicType HEURISTIC_TYPE = HeuristicType.EUCLIDEAN;
        public static boolean PREFER_STRAIGHT_PATHS = true;  // New config option
        public static double STRAIGHT_PATH_BONUS = 0.8;     // Cost multiplier for straight paths
        public static double DROP_COST_MULTIPLIER = 0.7; // Make drops more attractive than going around

        // New settings for enhanced movement and obstacle avoidance
        public static double SPEED_MULTIPLIER = 1.0; // Will be adjusted based on effects
        public static double JUMP_BOOST_MULTIPLIER = 1.0; // Will be adjusted based on effects
        public static int MAX_JUMP_HEIGHT = 1; // Base jump height
        public static int LOOK_AHEAD_DISTANCE = 5; // How far to look ahead for obstacles
        public static boolean ENABLE_ETHERWARP = false; // Support for etherwarp when available
        public static int CHUNK_SIZE = 50; // For handling very long distances
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





        // make the isValidPosition method static so it can be used without an instance
        public static boolean isValidPosition(BlockPos pos) {
            if (pos.getY() < 0 || pos.getY() >= 256) return false; // Y bounds check

            World world = Minecraft.getMinecraft().theWorld;
            if (world == null) return false;

            // Check for valid floor/support
            Block blockBelow = world.getBlockState(pos.down()).getBlock();
            Block blockAt = world.getBlockState(pos).getBlock();
            Block blockAbove = world.getBlockState(pos.up()).getBlock();

            // First check if the position itself is a valid block to be on
            boolean validGround = blockBelow.isNormalCube() ||
                    blockBelow instanceof net.minecraft.block.BlockStairs ||
                    blockBelow instanceof net.minecraft.block.BlockSlab ||
                    blockBelow instanceof net.minecraft.block.BlockFence ||
                    blockBelow instanceof net.minecraft.block.BlockWall ||
                    blockAt instanceof net.minecraft.block.BlockLadder ||
                    blockAt instanceof net.minecraft.block.BlockVine ||
                    blockAt instanceof net.minecraft.block.BlockFenceGate ||
                    blockAt instanceof net.minecraft.block.BlockIce ||
                    blockAt instanceof net.minecraft.block.BlockSnow ||
                    (Config.ALLOW_SWIMMING && blockAt.getMaterial().isLiquid());

            // Also check the block itself and one above for passability
            boolean hasHeadroom = (blockAt.isPassable(world, pos) ||
                    blockAt.getMaterial().isLiquid() ||
                    blockAt instanceof net.minecraft.block.BlockLadder ||
                    blockAt instanceof net.minecraft.block.BlockVine) &&
                    (blockAbove.isPassable(world, pos.up()) ||
                            blockAbove.getMaterial().isLiquid());

            // Special case: if we're checking goal position, be more lenient
            boolean isOnOrNearGround = validGround;
            if (!isOnOrNearGround) {
                // Check blocks around for valid ground in case we're near an edge
                for (int dx = -1; dx <= 1; dx++) {
                    for (int dz = -1; dz <= 1; dz++) {
                        if (dx == 0 && dz == 0) continue;
                        BlockPos checkPos = pos.add(dx, -1, dz);
                        Block checkBlock = world.getBlockState(checkPos).getBlock();
                        if (checkBlock.isNormalCube() ||
                                checkBlock instanceof net.minecraft.block.BlockStairs ||
                                checkBlock instanceof net.minecraft.block.BlockSlab) {
                            isOnOrNearGround = true;
                            break;
                        }
                    }
                }
            }

            // Debug output for invalid positions
            if (Config.DEBUG_ENABLED && (!isOnOrNearGround || !hasHeadroom)) {
                System.out.println("Invalid position at " + pos +
                        ": ground=" + isOnOrNearGround +
                        ", headroom=" + hasHeadroom +
                        ", block=" + blockAt.getLocalizedName() +
                        ", below=" + blockBelow.getLocalizedName() +
                        ", above=" + blockAbove.getLocalizedName());
            }

            return isOnOrNearGround && hasHeadroom;
        }
        public static boolean isPassable(BlockPos pos) {
            World world = Minecraft.getMinecraft().theWorld;
            if (world == null) return false;
            Block block = world.getBlockState(pos).getBlock();
            return block.isPassable(world, pos);
        }

        public static boolean isAirOrPassable(BlockPos pos) {
            Block block = Minecraft.getMinecraft().theWorld.getBlockState(pos).getBlock();
            return block.isAir(Minecraft.getMinecraft().theWorld, pos);
        }
        public static boolean isClimbable(BlockPos pos) {
            Block block = Minecraft.getMinecraft().theWorld.getBlockState(pos).getBlock();
            return block instanceof BlockLadder || block instanceof BlockStairs;
        }
        public static boolean isNearWall(BlockPos pos) {
            World world = Minecraft.getMinecraft().theWorld;
            for (int dx = -1; dx <= 1; dx++) {
                for (int dz = -1; dz <= 1; dz++) {
                    if (dx == 0 && dz == 0) continue; // Skip self
                    BlockPos neighbor = pos.add(dx, 0, dz);
                    Block block = world.getBlockState(neighbor).getBlock();
                    if (block.isNormalCube()) return true;
                }
            }
            return false;
        }

        public static boolean isNearObstacle(BlockPos pos) {
            World world = Minecraft.getMinecraft().theWorld;
            for (int dx = -1; dx <= 1; dx++) {
                for (int dz = -1; dz <= 1; dz++) {
                    if (dx == 0 && dz == 0) continue; // Skip self
                    BlockPos neighbor = pos.add(dx, 0, dz);
                    Block block = world.getBlockState(neighbor).getBlock();
                    if (!block.isAir(world, neighbor) && !block.isPassable(world, pos)) return true;
                }
            }
            return false;
        }


        // create calculateHeuristic method to calculate heuristic cost based on the heuristic type
        public static double calculateHeuristic(BlockPos start, BlockPos goal) {
            switch (Config.HEURISTIC_TYPE) {
                case MANHATTAN:
                    return Math.abs(start.getX() - goal.getX()) + Math.abs(start.getZ() - goal.getZ());
                case EUCLIDEAN:
                    return Math.sqrt(start.distanceSq(goal));
                case DIAGONAL:
                    int dx = Math.abs(start.getX() - goal.getX());
                    int dz = Math.abs(start.getZ() - goal.getZ());
                    return Math.max(dx, dz) + (Math.sqrt(2) - 1) * Math.min(dx, dz);
                default:
                    throw new IllegalArgumentException("Unknown heuristic type: " + Config.HEURISTIC_TYPE);
            }
        }
        // create hasObstaclesBetween method to check if there are obstacles between two positions
        public static boolean hasObstaclesBetween(BlockPos from, BlockPos to) {
            World world = Minecraft.getMinecraft().theWorld;
            int dx = to.getX() - from.getX();
            int dz = to.getZ() - from.getZ();
            int steps = Math.max(Math.abs(dx), Math.abs(dz));
            double stepX = dx / (double) steps;
            double stepZ = dz / (double) steps;

            for (int i = 0; i <= steps; i++) {
                BlockPos checkPos = from.add((int) (stepX * i), 0, (int) (stepZ * i));
                Block block = world.getBlockState(checkPos).getBlock();
                if (!block.isAir(world, checkPos) && !block.isPassable(world, checkPos)) {
                    return false;
                }
            }
            return true;
        }
        // create tryNeighbor method to attempt adding a neighbor node
        /**
         * Attempts to add a neighbor node using the given movement type.
         *
         * @param neighbors   the list to add to
         * @param current     the current node
         * @param neighborPos the position of the potential neighbor
         * @param goal        the goal position (for heuristic)
         * @param movementType the type of movement used to get here
         */
        private void tryNeighbor(
                List<PathNode> neighbors,
                PathNode current,
                BlockPos neighborPos,
                BlockPos goal,
                MovementType movementType
        ) {
            // 1) Valid position?
            if (!isValidPosition(neighborPos)) return;

            // 2) Headroom check
            if (!hasHeadroom(neighborPos)) return;

            // 3) Movement type must be allowed
            if (movementType == null) return;

            // 4) Compute costs
            double stepDistance = Math.sqrt(current.pos.distanceSq(neighborPos));
            double gCost = current.gCost + movementType.cost * stepDistance;
            double hCost = calculateHeuristic(neighborPos, goal);

            // 5) Construct and add the new node
            PathNode neighborNode = new PathNode(neighborPos, current, gCost, hCost, movementType);
            neighbors.add(neighborNode);
        }


        public static boolean hasHeadroom(BlockPos pos) {
            World world = Minecraft.getMinecraft().theWorld;
            for (int y = 0; y < 2; y++) {
                BlockPos checkPos = pos.up(y);
                Block block = world.getBlockState(checkPos).getBlock();
                if (!block.isAir(world, checkPos) && !block.isPassable(world, checkPos)) return false;
            }
            return true;
        }

        public static boolean hasObstacle(BlockPos from, BlockPos to, int maxHeight) {
            World world = Minecraft.getMinecraft().theWorld;

            // Null checks
            if (world == null || from == null || to == null) {
                return true;
            }

            // Check if chunks are loaded
            if (!world.isBlockLoaded(from) || !world.isBlockLoaded(to)) {
                return true;
            }

            int dx = to.getX() - from.getX();
            int dz = to.getZ() - from.getZ();
            int dy = to.getY() - from.getY();
            int steps = Math.max(Math.abs(dx), Math.abs(dz));

            // Prevent division by zero
            if (steps == 0) {
                return false;
            }

            double stepX = dx / (double) steps;
            double stepZ = dz / (double) steps;
            double stepY = dy / (double) steps;

            for (int i = 0; i <= steps; i++) {
                BlockPos checkPos = from.add((int) (stepX * i), (int) (stepY * i), (int) (stepZ * i));

                // Check if chunk is loaded for this position
                if (!world.isBlockLoaded(checkPos)) {
                    return true;
                }

                // Check vertical clearance (player height = 2 blocks)
                for (int y = 0; y < Math.max(2, maxHeight); y++) {
                    BlockPos currentPos = checkPos.up(y);

                    // Bounds check
                    if (currentPos.getY() < 0 || currentPos.getY() > 255) {
                        return true;
                    }

                    IBlockState blockState = world.getBlockState(currentPos);
                    Block block = blockState.getBlock();

                    // Basic solid block check
                    if (!block.isAir(world, currentPos) && !block.isPassable(world, currentPos)) {
                        // Check if block has custom collision bounds
                        AxisAlignedBB blockBounds = block.getCollisionBoundingBox(world, currentPos, blockState);
                        if (blockBounds != null) {
                            return true;
                        }
                    }

                    // Liquid checks
                    if (block instanceof BlockLiquid){
                        return true; // Treat all liquids as obstacles
                    }

                    // Specific block type checks
                    if (block instanceof BlockFire ||
                            block instanceof BlockStaticLiquid) {
                        return true;
                    }

                    // Web/cobweb check (slows movement significantly)
                    if (block == Blocks.web) {
                        return true;
                    }

                    // Partial blocks that might block movement
                    if (block instanceof BlockSlab &&
                            blockState.getValue(BlockSlab.HALF) == BlockSlab.EnumBlockHalf.TOP && y == 0) {
                        return true; // Top slabs at foot level
                    }

                    // Stairs are passable - don't block them

                    // Fence and wall checks
                    if (block instanceof BlockFence ||
                            block instanceof BlockWall ||
                            block instanceof BlockFenceGate) {
                        return true;
                    }

                    // Door checks (closed doors are obstacles)
                    if (block instanceof BlockDoor) {
                        if (!blockState.getValue(BlockDoor.OPEN)) {
                            return true;
                        }
                    }

                    // Trapdoor checks
                    if (block instanceof BlockTrapDoor) {
                        if (!blockState.getValue(BlockTrapDoor.OPEN)) {
                            return true;
                        }
                    }

                    // Pressure plate and redstone component checks
                    if (block instanceof BlockPressurePlate ||
                            block instanceof BlockRedstoneWire ||
                            block instanceof BlockTripWire) {
                        // These might not block movement but could trigger unwanted effects
                        return true;
                    }
                }

                // Diagonal movement checks (improved)
                if (Math.abs(dx) > 0 && Math.abs(dz) > 0) {
                    // Check both adjacent blocks for diagonal movement
                    BlockPos diagCheckX = checkPos.add(Integer.signum(dx), 0, 0);
                    BlockPos diagCheckZ = checkPos.add(0, 0, Integer.signum(dz));

                    if (world.isBlockLoaded(diagCheckX) && world.isBlockLoaded(diagCheckZ)) {
                        for (int y = 0; y < Math.max(2, maxHeight); y++) {
                            BlockPos diagPosX = diagCheckX.up(y);
                            BlockPos diagPosZ = diagCheckZ.up(y);

                            IBlockState diagStateX = world.getBlockState(diagPosX);
                            IBlockState diagStateZ = world.getBlockState(diagPosZ);
                            Block diagBlockX = diagStateX.getBlock();
                            Block diagBlockZ = diagStateZ.getBlock();

                            // Check if diagonal movement is blocked by adjacent solid blocks
                            if ((!diagBlockX.isAir(world, diagPosX) && !diagBlockX.isPassable(world, diagPosX)) ||
                                    (!diagBlockZ.isAir(world, diagPosZ) && !diagBlockZ.isPassable(world, diagPosZ))) {

                                // Additional checks for collision bounds
                                AxisAlignedBB boundsX = diagBlockX.getCollisionBoundingBox(world, diagPosX, diagStateX);
                                AxisAlignedBB boundsZ = diagBlockZ.getCollisionBoundingBox(world, diagPosZ, diagStateZ);

                                if (boundsX != null || boundsZ != null) {
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
            return false;
        }

        //make an addtional check for obstacles in the direction of movement becuase it can break if theres only 1 block and we just stand still without jumping over it
        public static boolean canMoveDiagonally(BlockPos from, BlockPos to) {
            World world = Minecraft.getMinecraft().theWorld;
            if (world == null) return false;

            // Check if the diagonal move is valid
            if (!isValidPosition(to)) return false;

            // Check for obstacles in the diagonal direction
            int dx = to.getX() - from.getX();
            int dz = to.getZ() - from.getZ();
            if (Math.abs(dx) != Math.abs(dz)) return false; // Must be a true diagonal

            // Check both blocks in the diagonal path
            BlockPos checkPos1 = from.add(dx, 0, 0);
            BlockPos checkPos2 = from.add(0, 0, dz);
            return isValidPosition(checkPos1) && isValidPosition(checkPos2);
        }

        public static boolean isDiagonalMove(BlockPos from, BlockPos to) {
            return Math.abs(from.getX() - to.getX()) == 1 && Math.abs(from.getZ() - to.getZ()) == 1;
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

        //check if theres a wall if there is a wall we try to find a gap if none is found we return a longer path that goes around the wall
        if (hasObstacle(start, goal, Config.MAX_JUMP_HEIGHT)) {
            List<BlockPos> gaps = findWallGap(start, goal);
            if (!gaps.isEmpty()) {
                // If we found a gap, use it as the new start
                start = gaps.get(0);
            } else {
                // No gaps found, return a longer path that goes around the wall
                return new PathResult(new ArrayList<>(), false, 0, 0, 0, "No gap found in wall");
            }
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
        BlockPos       pos       = current.pos;

        int[][] cardinalDirs = {{0,1}, {1,0}, {0,-1}, {-1,0}};
        for (int[] dir : cardinalDirs) {
            BlockPos flatPos = pos.add(dir[0], 0, dir[1]);
            IBlockState  state    = world.getBlockState(flatPos);
            Block        block    = state.getBlock();

            // ─── 1) STAIRS ───────────────────────────────────
            if (block instanceof BlockStairs) {
                BlockPos stepUp = flatPos.up();
                if (isValidPosition(stepUp) && hasHeadroom(stepUp)) {
                    current.tryNeighbor(neighbors, current, stepUp, goal, MovementType.CLIMB);
                }
                continue;  // *** skip any jump‐over here! ***
            }

            // ─── 2) TOP‐HALF SLABS ────────────────────────────
            if (block instanceof BlockSlab && !((BlockSlab)block).isDouble()) {
                BlockSlab.EnumBlockHalf half = state.getValue(BlockSlab.HALF);
                if (half == BlockSlab.EnumBlockHalf.TOP) {
                    BlockPos slabTop = flatPos.up();
                    if (isValidPosition(slabTop) && hasHeadroom(slabTop)) {
                        current.tryNeighbor(neighbors, current, slabTop, goal, MovementType.WALK);
                    }
                    continue;
                }
            }

            // ─── 3) STRAIGHT‐PATH BONUS ──────────────────────
            if (current.parent != null) {
                int prevDx = pos.getX() - current.parent.pos.getX();
                int prevDz = pos.getZ() - current.parent.pos.getZ();
                if (prevDx == dir[0] && prevDz == dir[1] && Config.PREFER_STRAIGHT_PATHS) {
                    PathNode straightNode = new PathNode(
                            flatPos,
                            current,
                            current.gCost + MovementType.WALK.cost * Config.STRAIGHT_PATH_BONUS,
                            calculateHeuristic(flatPos, goal),
                            MovementType.WALK
                    );
                    if (isValidPosition(flatPos) && hasHeadroom(flatPos)) {
                        neighbors.add(straightNode);
                        continue;
                    }
                }
            }

            // ─── 4) FLAT WALK ───────────────────────────────
            current.tryNeighbor(neighbors, current, flatPos, goal, MovementType.WALK);

            // ─── 5) JUMP OVER SINGLE‐BLOCK OBSTACLE ──────────
            // Only if the obstacle isn’t a stair (so you can’t jump the whole staircase in one go)
            BlockPos ahead    = flatPos;
            BlockPos above    = ahead.up();
            IBlockState obsSt = world.getBlockState(ahead);
            if (!(obsSt.getBlock() instanceof BlockStairs)
                    && !isValidPosition(ahead)
                    && isValidPosition(above)
                    && hasHeadroom(above)) {
                current.tryNeighbor(neighbors, current, above, goal, MovementType.JUMP);
            }
        }

        // ─── 6) DIAGONALS ────────────────────────────────
        if (neighbors.size() < 2) {
            int[][] diagonalDirs = {{1,1},{1,-1},{-1,1},{-1,-1}};
            for (int[] dir : diagonalDirs) {
                BlockPos diagPos = pos.add(dir[0], 0, dir[1]);
                if (canMoveDiagonally(pos, diagPos)) {
                    current.tryNeighbor(neighbors, current, diagPos, goal, MovementType.DIAGONAL);
                }
            }
        }

        // ─── 7) DROP DOWN ────────────────────────────────
        BlockPos down = pos.down();
        if (isValidPosition(down)) {
            current.tryNeighbor(neighbors, current, down, goal, MovementType.FALL);
        }

        return neighbors;
    }

    private boolean hasWallInDirection(BlockPos pos, int dx, int dz) {
        int wallHeight = 0;
        for (int y = 0; y < 3; y++) {
            BlockPos check = pos.add(dx, y, dz);
            if (world.getBlockState(check).getBlock().isNormalCube()) {
                wallHeight++;
            }
        }
        return wallHeight >= 2;
    }

    //create handleObstacleAvoidance to handle             handleObstacleAvoidance(player, target);
    public static boolean handleObstacleAvoidance(EntityPlayer player, BlockPos target) {
        BlockPos playerPos = player.getPosition();
        if (playerPos.equals(target)) return true; // Already at target

        // Check if we can move directly to the target
        if (PathNode.hasObstaclesBetween(playerPos, target)) {
            // If there are obstacles, try to find a gap
            List<BlockPos> gaps = findWallGap(playerPos, target);
            if (!gaps.isEmpty()) {
                // If we found a gap, move to the closest one
                BlockPos closestGap = gaps.get(0);
                player.setPositionAndUpdate(closestGap.getX(), closestGap.getY(), closestGap.getZ());
                return true;
            }
            // If no gaps found, return false
            return false;
        }
        // No obstacles, we can move directly
        player.setPositionAndUpdate(target.getX(), target.getY(), target.getZ());
        return true;
    }


    private static List<BlockPos> findWallGap(BlockPos start, BlockPos end) {
        List<BlockPos> potentialPoints = new ArrayList<>();
        double dx = end.getX() - start.getX();
        double dz = end.getZ() - start.getZ();
        double length = Math.sqrt(dx * dx + dz * dz);
        // Normalize direction vector
        double dirX = dx / length;
        double dirZ = dz / length;

        World world = Minecraft.getMinecraft().theWorld;
        if (world == null) return potentialPoints;

        // Search in a wider area for gaps
        for (int dist = 1; dist <= 5; dist++) {
            for (int offset = -3; offset <= 3; offset++) {
                // Check perpendicular to the direction
                double perpX = -dirZ * offset;
                double perpZ = dirX * offset;

                BlockPos check = new BlockPos(
                        start.getX() + dirX * dist + perpX,
                        start.getY(),
                        start.getZ() + dirZ * dist + perpZ
                );

                // Look for 2-block high gaps
                if (isValidGapPoint(check)) {
                    potentialPoints.add(check);
                }

                // Also check different Y levels
                for (int y = -1; y <= 2; y++) {
                    BlockPos elevated = check.add(0, y, 0);
                    if (isValidGapPoint(elevated)) {
                        potentialPoints.add(elevated);
                    }
                }
            }
        }

        // Sort points by distance to target
        potentialPoints.sort((a, b) -> Double.compare(
                a.distanceSq(end),
                b.distanceSq(end)
        ));

        return potentialPoints;
    }

    private static boolean isValidGapPoint(BlockPos pos) {
        World world = Minecraft.getMinecraft().theWorld;
        if (world == null) return false;

        // Must have space to stand and move
        if (!isAirOrPassable(pos) || !isAirOrPassable(pos.up())) {
            return false;
        }

        // Must have solid ground or climbable
        Block below = world.getBlockState(pos.down()).getBlock();
        if (!below.isNormalCube() && !isClimbable(pos)) {
            return false;
        }

        // Check for clearance on at least one side
        boolean hasPath = false;
        for (int[] dir : new int[][]{{1,0}, {-1,0}, {0,1}, {0,-1}}) {
            BlockPos side = pos.add(dir[0], 0, dir[1]);
            if (isAirOrPassable(side) && isAirOrPassable(side.up())) {
                hasPath = true;
                break;
            }
        }

        return hasPath;
    }

    private static MovementType getMovementType(BlockPos from, BlockPos to) {
        int dy = to.getY() - from.getY();

        // First check for vertical movement
        if (dy > 0) {
            if (dy > Config.MAX_JUMP_HEIGHT) {
                // Check if we can climb
                if (isClimbable(to) || isClimbable(from)) {
                    return MovementType.CLIMB;
                }
                return null;
            }
            return MovementType.JUMP;
        } else if (dy < 0) {
            if (-dy > Config.MAX_DROP_HEIGHT) return null;
            return MovementType.FALL;
        }

        // Then check horizontal movement
        if (isDiagonalMove(from, to)) {
            // Extra validation for diagonal moves
            if (!canMoveDiagonally(from, to)) return null;
            return MovementType.DIAGONAL;
        }

        return MovementType.WALK;
    }



    private static boolean canMoveDiagonally(BlockPos from, BlockPos to) {
        // Check both cardinal directions that make up the diagonal
        BlockPos xStep = new BlockPos(to.getX(), from.getY(), from.getZ());
        BlockPos zStep = new BlockPos(from.getX(), from.getY(), to.getZ());

        // Must have valid positions and head clearance in both directions
        return isValidPosition(xStep) && isValidPosition(zStep) &&
                hasHeadroom(xStep) && hasHeadroom(zStep) &&
                hasObstaclesBetween(from, xStep) && hasObstaclesBetween(from, zStep);
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
        return !PathNode.hasObstacle(from, to, 2);
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
            GL11.glColor3f(1.0f, 0.5f, 0.0f); // Orange for path

            for (BlockPos pos : currentPath) {
                // Draw block outline
                GL11.glBegin(GL11.GL_LINE_LOOP);
                // Bottom face
                GL11.glVertex3d(pos.getX(), pos.getY(), pos.getZ());
                GL11.glVertex3d(pos.getX() + 1, pos.getY(), pos.getZ());
                GL11.glVertex3d(pos.getX() + 1, pos.getY(), pos.getZ() + 1);
                GL11.glVertex3d(pos.getX(), pos.getY(), pos.getZ() + 1);
                GL11.glEnd();

                // Top face
                GL11.glBegin(GL11.GL_LINE_LOOP);
                GL11.glVertex3d(pos.getX(), pos.getY() + 1, pos.getZ());
                GL11.glVertex3d(pos.getX() + 1, pos.getY() + 1, pos.getZ());
                GL11.glVertex3d(pos.getX() + 1, pos.getY() + 1, pos.getZ() + 1);
                GL11.glVertex3d(pos.getX(), pos.getY() + 1, pos.getZ() + 1);
                GL11.glEnd();

                // Vertical edges
                GL11.glBegin(GL11.GL_LINES);
                GL11.glVertex3d(pos.getX(), pos.getY(), pos.getZ());
                GL11.glVertex3d(pos.getX(), pos.getY() + 1, pos.getZ());

                GL11.glVertex3d(pos.getX() + 1, pos.getY(), pos.getZ());
                GL11.glVertex3d(pos.getX() + 1, pos.getY() + 1, pos.getZ());

                GL11.glVertex3d(pos.getX() + 1, pos.getY(), pos.getZ() + 1);
                GL11.glVertex3d(pos.getX() + 1, pos.getY() + 1, pos.getZ() + 1);

                GL11.glVertex3d(pos.getX(), pos.getY(), pos.getZ() + 1);
                GL11.glVertex3d(pos.getX(), pos.getY() + 1, pos.getZ() + 1);
                GL11.glEnd();
            }
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
            double s = size * 10;

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