package com.shadow.ShadowAddons.utils.pathfinding;

import com.shadow.ShadowAddons.utils.RotationsUtils.PathFindRotation;
import net.minecraft.block.*;
import net.minecraft.block.material.Material;
import net.minecraft.client.Minecraft;
import net.minecraft.client.settings.KeyBinding;
import net.minecraft.entity.EntityLivingBase;
import net.minecraft.init.Blocks;
import net.minecraft.util.BlockPos;
import net.minecraft.util.EnumFacing;
import net.minecraft.util.MovingObjectPosition;
import net.minecraft.util.Vec3;
import net.minecraft.world.World;
import net.minecraftforge.client.event.RenderWorldLastEvent;
import net.minecraftforge.common.MinecraftForge;
import net.minecraftforge.fml.common.eventhandler.SubscribeEvent;
import org.lwjgl.opengl.GL11;

import java.util.*;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class old {
    private final World world;
    private final MovementController movementController;
    private final EnvironmentScanner environmentScanner;

    // The current path (null if none has been found yet)
    private Path currentPath;

    // Whether the last attempt returned "no path found"
    private boolean noPathFound = false;

    // Shared single-threaded executor to run A* in the background
    private final ExecutorService executor = Executors.newSingleThreadExecutor();

    // Debug flag (prints to console if enabled)
    public boolean debugEnabled = false;

    // Block-specific movement costs
    private final Map<Block, Double> blockCosts = new HashMap<>();

    // To throttle recalculation when stuck
    private long lastRecalculationTime = 0L;

    private static final int MAX_ITERATIONS = 5000; // Limit pathfinding iterations
    private static final double HEURISTIC_WEIGHT = 1.5; // Weight for heuristic to make it more aggressive
    private static final int RECALCULATION_COOLDOWN = 500; // 0.5 seconds cooldown between recalculations (was 0, reverted for stability)
    private static final int MAX_PATHS_TO_CHECK = 100; // Increased from 5 to 10 paths
    private static final double PATH_DEVIATION_FACTOR = 2.0; // Increased from 1.5 to 2.0 for more path variety
    private static final int MAX_PATHFINDING_ATTEMPTS = 50; // Maximum number of attempts to find a working path
    private static final int OBSTACLE_CHECK_RANGE = 4;
    private static final int OBSTACLE_CHECK_HEIGHT = 2;
    private static final int ACCESSIBILITY_CHECK_RANGE = 2;

    // Note: A small RECALCULATION_COOLDOWN is important for stability and preventing excessive recalculations.

    private int currentPathfindingAttempt = 0;
    private BlockPos lastStartPos = null;
    private BlockPos lastGoalPos = null;

    public old(World world) {
        this.world = world;
        this.movementController = new MovementController();
        this.environmentScanner = new EnvironmentScanner(world);
        initializeBlockCosts();
        // Register this class to the MinecraftForge event bus for rendering
        MinecraftForge.EVENT_BUS.register(this);
    }

    private void initializeBlockCosts() {
        // Default costs for common blocks
        blockCosts.put(Blocks.soul_sand, 2.0);
        blockCosts.put(Blocks.water, 1.5);
        blockCosts.put(Blocks.lava, 10.0);
    }

    /**
     * Asynchronously calculate a path from `start` to `goal`. Once done,
     * sets `currentPath` (or flags `noPathFound`) and posts a client chat msg.
     */
    public void calculatePathAsync(BlockPos start, BlockPos goal) {
        lastStartPos = start;
        lastGoalPos = goal;
        currentPathfindingAttempt = 0;
        executor.submit(() -> {
            Path path = calculatePathSync(start, goal);
            if (path == null && currentPathfindingAttempt < MAX_PATHFINDING_ATTEMPTS) {
                // If no path found, try again with different parameters
                currentPathfindingAttempt++;
                calculatePathAsync(start, goal);
            } else {
                currentPath = path;
                noPathFound = (path == null);
            }
        });
    }

    /**
     * Synchronous A* implementation. Returns a Path if one exists, or null if none.
     */
    public Path calculatePathSync(BlockPos start, BlockPos goal) {
        List<Path> possiblePaths = new ArrayList<>();
        PriorityQueue<PathNode> openSet = new PriorityQueue<>();
        Map<BlockPos, PathNode> allNodes = new HashMap<>();
        int iterations = 0;

        // Try to find multiple paths with different parameters
        for (int attempt = 0; attempt < MAX_PATHS_TO_CHECK; attempt++) {
            openSet.clear();
            allNodes.clear();
            iterations = 0;

            // Adjust heuristic modifier based on attempt number
            double heuristicModifier = 1.0 + (attempt * 0.2); // Gradually increase heuristic weight
            double deviationFactor = PATH_DEVIATION_FACTOR * (1.0 + (attempt * 0.1)); // Gradually increase deviation

            PathNode startNode = new PathNode(start, null, 0.0, heuristic(start, goal) * heuristicModifier);
            openSet.add(startNode);
            allNodes.put(start, startNode);

            while (!openSet.isEmpty() && iterations < MAX_ITERATIONS) {
                iterations++;
                PathNode current = openSet.poll();
                current.setClosed(true);

                if (current.pos.equals(goal)) {
                    Path path = reconstructPath(current);
                    if (isPathAccessible(path)) {
                        possiblePaths.add(path);
                        if (attempt == 0) {
                            return path; // Return first accessible path immediately
                        }
                    }
                    break;
                }

                List<BlockPos> neighbors = getOptimizedNeighbors(current.pos, goal, deviationFactor);
                for (BlockPos neighbor : neighbors) {
                    if (!environmentScanner.isChunkLoaded(neighbor)) continue;

                    PathNode neighborNode = allNodes.computeIfAbsent(
                            neighbor,
                            pos -> new PathNode(pos, Double.MAX_VALUE, heuristic(pos, goal) * heuristicModifier)
                    );

                    if (neighborNode.isClosed()) continue;

                    double newCost = current.gCost + getMovementCost(current.pos, neighbor);
                    if (newCost < neighborNode.gCost) {
                        neighborNode.parent = current;
                        neighborNode.gCost = newCost;
                        neighborNode.fCost = newCost + (neighborNode.hCost * HEURISTIC_WEIGHT);
                        neighborNode.action = determineAction(current.pos, neighbor);

                        if (!openSet.contains(neighborNode)) {
                            openSet.add(neighborNode);
                        }
                    }
                }
            }
        }

        // If we found multiple paths, choose the best one
        if (!possiblePaths.isEmpty()) {
            return selectBestPath(possiblePaths);
        }

        // If no accessible paths found, try to find the closest accessible point
        PathNode closest = findClosestAccessibleNode(allNodes.values(), goal);
        if (closest != null) {
            return reconstructPath(closest);
        }

        return null;
    }

    private Path reconstructPath(PathNode node) {
        List<PathNode> pathList = new ArrayList<>();
        while (node != null) {
            pathList.add(node);
            node = node.parent;
        }
        Collections.reverse(pathList);
        return new Path(simplifyPath(pathList), pathList.get(0).pos, pathList.get(pathList.size() - 1).pos);
    }

    private List<PathNode> simplifyPath(List<PathNode> rawPath) {
        if (rawPath.size() < 3) {
            return rawPath;
        }

        List<PathNode> simplified = new ArrayList<>();
        simplified.add(rawPath.get(0)); // Always keep start

        int i = 0;
        while (i < rawPath.size() - 1) {
            PathNode current = rawPath.get(i);
            simplified.add(current);

            // Look ahead to find the furthest node we can reach directly
            int furthest = i + 1;
            for (int j = i + 2; j < rawPath.size(); j++) {
                PathNode target = rawPath.get(j);

                // Don't simplify if there's a significant action or height change
                if (target.action == ActionType.JUMP ||
                        target.action == ActionType.FALL ||
                        Math.abs(target.pos.getY() - current.pos.getY()) > 1) {
                    break;
                }

                // Check if we can move directly from current to target
                if (environmentScanner.hasLineOfSight(current.pos, target.pos) &&
                        canMoveBetween(current.pos, target.pos)) {
                    furthest = j;
                } else {
                    break;
                }
            }

            i = furthest;
        }

        // Always keep the final node
        if (!simplified.contains(rawPath.get(rawPath.size() - 1))) {
            simplified.add(rawPath.get(rawPath.size() - 1));
        }

        return simplified;
    }
    private double getMovementCost(BlockPos from, BlockPos to) {
        double base = from.distanceSq(to); // Use squared distance for performance
        base = Math.sqrt(base); // Convert back to actual distance

        Block toBlock = world.getBlockState(to).getBlock();
        double blockMultiplier = blockCosts.getOrDefault(toBlock, 1.0);

        // Height penalties
        int dy = to.getY() - from.getY();
        if (dy > 0) {
            base += dy * 0.5; // Climbing penalty
        } else if (dy < 0) {
            base += Math.abs(dy) * 0.1; // Small falling penalty
        }

        // Diagonal movement penalty
        if (Math.abs(to.getX() - from.getX()) > 0 && Math.abs(to.getZ() - from.getZ()) > 0) {
            base *= 1.2; // Slight diagonal penalty
        }

        // Liquid penalty
        if (environmentScanner.isLiquid(to)) {
            base *= 1.8;
        }

        // Dangerous block penalty
        if (environmentScanner.isDangerous(to)) {
            base *= 5.0;
        }

        return base * blockMultiplier;
    }

    private ActionType determineAction(BlockPos current, BlockPos next) {
        int dy = next.getY() - current.getY();
        if (dy == 1) return ActionType.JUMP;
        if (dy == -1) return ActionType.FALL;
        return ActionType.WALK;
    }

    private double heuristic(BlockPos a, BlockPos b) {
        // Manhattan distance for better performance
        return Math.abs(a.getX() - b.getX()) + 
               Math.abs(a.getY() - b.getY()) + 
               Math.abs(a.getZ() - b.getZ());
    }

    private List<BlockPos> getOptimizedNeighbors(BlockPos current, BlockPos goal, double deviationFactor) {
        List<BlockPos> neighbors = new ArrayList<>();

        // Calculate direction to goal
        int dx = Integer.signum(goal.getX() - current.getX());
        int dz = Integer.signum(goal.getZ() - current.getZ());
        int dy = Integer.signum(goal.getY() - current.getY());

        // Priority order: direct path, then adjacent directions
        int[][] directions = {
                {dx, dz}, {dx, 0}, {0, dz}, // Direct and semi-direct
                {-dx, 0}, {0, -dz}, {-dx, -dz}, // Opposite directions
                {1, 0}, {-1, 0}, {0, 1}, {0, -1}, // Cardinal directions
                {1, 1}, {1, -1}, {-1, 1}, {-1, -1} // Diagonals
        };

        for (int[] dir : directions) {
            // Try different Y levels for each horizontal direction
            for (int yOffset = -1; yOffset <= 2; yOffset++) {
                // Prefer movements toward goal Y level
                int actualY = yOffset;
                if (dy != 0) {
                    actualY = dy > 0 ? Math.max(yOffset, 0) : Math.min(yOffset, 0);
                }

                BlockPos neighbor = current.add(dir[0], actualY, dir[1]);

                // Skip if too far from goal (deviation control)
                double distToGoal = neighbor.distanceSq(goal);
                double currentDistToGoal = current.distanceSq(goal);
                if (distToGoal > currentDistToGoal * deviationFactor) continue;

                if (isValidNeighbor(current, neighbor)) {
                    neighbors.add(neighbor);
                }
            }
        }

        return neighbors;
    }

    private boolean isValidNeighbor(BlockPos current, BlockPos neighbor) {
        int dy = neighbor.getY() - current.getY();

        // Check if chunk is loaded first
        if (!environmentScanner.isChunkLoaded(neighbor)) return false;

        // Allow reasonable vertical movement
        if (Math.abs(dy) > 2) return false; // Don't allow more than 2 blocks vertical change

        // For falling (negative dy)
        if (dy < 0) {
            // Allow falling if target is not solid and not dangerous
            return !environmentScanner.isSolid(neighbor) &&
                    !environmentScanner.isDangerous(neighbor) &&
                    environmentScanner.isWalkable(neighbor.down()); // Need solid ground below
        }

        // For horizontal movement (dy == 0)
        if (dy == 0) {
            return environmentScanner.isWalkable(neighbor) &&
                    !environmentScanner.hasObstacleBetween(current, neighbor);
        }

        // For upward movement (dy > 0)
        if (dy > 0) {
            // Check if it's the goal position
            if (lastGoalPos != null && neighbor.equals(lastGoalPos)) {
                return environmentScanner.isWalkable(neighbor) &&
                        !environmentScanner.hasObstacleBetween(current, neighbor);
            }

            // Check for stairs or valid jump
            if (environmentScanner.isValidStairStep(current, neighbor) ||
                    (dy == 1 && environmentScanner.canJumpUpTo(current, neighbor))) {
                return environmentScanner.isWalkable(neighbor) &&
                        !environmentScanner.hasObstacleBetween(current, neighbor);
            }

            return false; // Can't move up more than 1 block without stairs
        }

        return false;
    }

    private EnumFacing getDirectionToGoal(BlockPos current) {
        if (currentPath == null || currentPath.nodes.isEmpty()) {
            return EnumFacing.NORTH;
        }
        
        BlockPos goal = currentPath.nodes.get(currentPath.nodes.size() - 1).pos;
        int dx = goal.getX() - current.getX();
        int dz = goal.getZ() - current.getZ();
        
        if (Math.abs(dx) > Math.abs(dz)) {
            return dx > 0 ? EnumFacing.EAST : EnumFacing.WEST;
        } else {
            return dz > 0 ? EnumFacing.SOUTH : EnumFacing.NORTH;
        }
    }

    private PathNode findClosestNodeToGoal(Collection<PathNode> nodes, BlockPos goal) {
        PathNode closest = null;
        double minDist = Double.MAX_VALUE;
        
        for (PathNode node : nodes) {
            double dist = node.pos.distanceSq(goal);
            if (dist < minDist) {
                minDist = dist;
                closest = node;
            }
        }
        
        return closest;
    }

    /**
     * Called every client tick (if a path is active). Moves `entity` along the path.
     */
    public void followPath(EntityLivingBase entity) {
        if (currentPath == null || currentPath.isComplete()) {
            return;
        }
        movementController.followPath(entity, currentPath);
        if (debugEnabled) {
            renderDebugOverlay();
        }
    }

    /**
     * If stuck for more than ~2 seconds (40 ticks), recalc a new path to the same goal.
     */
    public void recalculatePath(BlockPos currentPosition, BlockPos newGoal) {
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastRecalculationTime > RECALCULATION_COOLDOWN) {
            calculatePathAsync(currentPosition, newGoal);
            lastRecalculationTime = currentTime;
        }
    }

    @SubscribeEvent
    public void onRenderWorldLast(RenderWorldLastEvent event) {
        System.out.println("Rendering World Last Event Triggered"); // Debug print
        if (debugEnabled && currentPath != null) {
            renderDebugOverlay();
        }
    }

    public void renderDebugOverlay() {
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

    // ----------------------------
    //       Inner Classes
    // ----------------------------

    private static class PathNode implements Comparable<PathNode> {
        final BlockPos pos;
        PathNode parent;
        double gCost;
        final double hCost;
        double fCost;
        ActionType action;
        boolean closed = false;

        PathNode(BlockPos pos, PathNode parent, double gCost, double hCost) {
            this.pos = pos;
            this.parent = parent;
            this.gCost = gCost;
            this.hCost = hCost;
            this.fCost = gCost + hCost;
        }

        PathNode(BlockPos pos, double gCost, double hCost) {
            this(pos, null, gCost, hCost);
        }

        public boolean isClosed() {
            return closed;
        }

        public void setClosed(boolean closed) {
            this.closed = closed;
        }

        @Override
        public int compareTo(PathNode other) {
            return Double.compare(this.fCost, other.fCost);
        }
    }

    public class Path {
        private final List<PathNode> nodes;
        private int currentIndex = 0;
        private boolean completed = false;
        private final BlockPos start;
        private final BlockPos goal;

        public Path(List<PathNode> nodes, BlockPos start, BlockPos goal) {
            this.nodes = nodes;
            this.start = start;
            this.goal = goal;
            // Only mark as complete if there are no nodes at all
            if (nodes.isEmpty()) {
                this.completed = true;
            }
        }

        public PathNode getNextNode() {
            if (isComplete()) return null;
            // Ensure we don't go out of bounds
            if (currentIndex >= nodes.size()) return null;
            return nodes.get(currentIndex);
        }

        public void advance() {
            currentIndex++;
            if (currentIndex >= nodes.size()) {
                completed = true;
            }
        }

        public boolean isComplete() {
            // Path is complete if all nodes visited OR if explicitly marked complete
            return completed || currentIndex >= nodes.size();
        }

        public boolean hasReachedGoal(EntityLivingBase entity) {
            if (goal == null) return false;
            // Check if the entity is within the block of the goal position and is close to the center
            BlockPos entityPos = new BlockPos(entity);
            boolean isInGoalBlock = entityPos.equals(goal);
            // Corrected: getDistanceSqToCenter needs a BlockPos
            boolean isCloseToCenter = entity.getDistanceSqToCenter(goal) < 1.0; // Check distance to block center

            return isInGoalBlock && isCloseToCenter; // Must be in the block AND close to center
        }

        public BlockPos getStart() {
            return start;
        }

        public BlockPos getGoal() {
            return goal;
        }
    }

    private class MovementController {
        private static final int STUCK_THRESHOLD = 15;
        private static final int WALL_CHECK_THRESHOLD = 30;
        private static final double MOVEMENT_THRESHOLD = 0.005;
        private static final double NODE_COMPLETION_DISTANCE = 1.2;
        private static final double FINAL_GOAL_STOP_DISTANCE_SQ = 0.5;
        private static final int MAX_RECALCULATION_ATTEMPTS = 3;
        private static final long RECALCULATION_COOLDOWN = 2000; // 2 seconds

        private int stuckTimer = 0;
        private BlockPos lastPosition = new BlockPos(0, 0, 0);
        private final Minecraft mc = Minecraft.getMinecraft();
        private int jumpCooldown = 0;
        private boolean isSprinting = false;
        private int sprintTicks = 0;
        private int wallCheckTimer = 0;
        private int recalculationAttempts = 0;
        private long lastRecalculationTime = 0;

        public void followPath(EntityLivingBase entity, Path path) {
            if (path == null || path.isComplete()) return;

            // Get current node and target node
            PathNode currentNode = path.getNextNode();
            if (currentNode == null) return;

            BlockPos targetNodePos = currentNode.pos;
            Vec3 targetNodeCenter = new Vec3(
                targetNodePos.getX() + 0.5,
                targetNodePos.getY() + (currentNode.action == ActionType.JUMP ? 1.0 : 0.0) + entity.getEyeHeight(),
                targetNodePos.getZ() + 0.5
            );

            // Use RotationUtils for smooth rotation
            PathFindRotation.RotationUtils.rotateToPosition(entity, targetNodeCenter);

            // Check if we're stuck
            BlockPos currentPos = new BlockPos(entity.posX, entity.posY, entity.posZ);
            if (currentPos.equals(lastPosition)) {
                stuckTimer++;
                if (stuckTimer > STUCK_THRESHOLD) {
                    // Check if we're surrounded by solid blocks
                    if (isSurroundedBySolidBlocks(entity)) {
                        // Force recalculation from current position
                        long currentTime = System.currentTimeMillis();
                        if (currentTime - lastRecalculationTime > RECALCULATION_COOLDOWN && 
                            recalculationAttempts < MAX_RECALCULATION_ATTEMPTS) {
                            recalculationAttempts++;
                            lastRecalculationTime = currentTime;
                            calculatePathAsync(currentPos, path.getGoal());
                            return;
                        }
                    }
                }
            } else {
                stuckTimer = 0;
                lastPosition = currentPos;
            }

            // Rest of the movement logic...

            // 2) Set movement keys (Forward is handled below based on facing direction)
            KeyBinding.setKeyBindState(mc.gameSettings.keyBindBack.getKeyCode(), false);
            KeyBinding.setKeyBindState(mc.gameSettings.keyBindLeft.getKeyCode(), false);
            KeyBinding.setKeyBindState(mc.gameSettings.keyBindRight.getKeyCode(), false);

            // Control forward movement based on facing direction and target
             boolean isFacingTarget = PathFindRotation.RotationUtils.isFacingPosition(entity, targetNodeCenter, 15.0f); // Increased tolerance slightly
             KeyBinding.setKeyBindState(mc.gameSettings.keyBindForward.getKeyCode(), isFacingTarget);


            // 3) Smart jumping logic
            if (jumpCooldown > 0) {
                jumpCooldown--;
                KeyBinding.setKeyBindState(mc.gameSettings.keyBindJump.getKeyCode(), false);
            } else {
                boolean shouldJump = shouldJump(entity, currentNode);
                KeyBinding.setKeyBindState(mc.gameSettings.keyBindJump.getKeyCode(), shouldJump);
                if (shouldJump) {
                    jumpCooldown = 8;
                }
            }

            // 4) Enhanced sprinting logic
            boolean canSprint = canSprint(entity, path);
            // Only sprint if facing the target direction reasonably well and moving forward
             if (canSprint && isFacingTarget) {
                if (!isSprinting) {
                    isSprinting = true;
                    sprintTicks = 0;
                }
                sprintTicks++;

                // Activate sprint key after a few ticks of moving forward and being able to sprint
                if (sprintTicks > 2) {
                    KeyBinding.setKeyBindState(mc.gameSettings.keyBindSprint.getKeyCode(), true);
                }
            } else {
                // Stop sprinting if can't sprint or not moving forward
                if (isSprinting) {
                    isSprinting = false;
                    sprintTicks = 0;
                    KeyBinding.setKeyBindState(mc.gameSettings.keyBindSprint.getKeyCode(), false);
                }
                 KeyBinding.setKeyBindState(mc.gameSettings.keyBindSprint.getKeyCode(), false); // Ensure sprint is off
            }

            // 5) Wall detection and stuck handling
             // Check for significant movement
            if (currentPos.distanceSq(lastPosition) < MOVEMENT_THRESHOLD) {
                if (++stuckTimer > STUCK_THRESHOLD) {
                     // More sophisticated stuck checks can be added here
                    if (isStuckOnWall(entity) || isSurroundedBySolidBlocks(entity)) { // Added isSurroundedBySolidBlocks check
                        KeyBinding.setKeyBindState(mc.gameSettings.keyBindJump.getKeyCode(), true);
                        jumpCooldown = 8;

                        if (++wallCheckTimer > WALL_CHECK_THRESHOLD) {
                            recalculatePath(currentPos, path.getGoal()); // Recalculate using the stored goal
                            wallCheckTimer = 0;
                        }
                    } else {
                        recalculatePath(currentPos, path.getGoal()); // Recalculate using the stored goal
                        wallCheckTimer = 0;
                    }
                    stuckTimer = 0;
                }
            } else {
                stuckTimer = 0;
                wallCheckTimer = 0;
                lastPosition = currentPos;
            }

            // 6) Advance to next node if close enough
            // Corrected: Use BlockPos targetNodePos for getDistanceSqToCenter
            if (entity.getDistanceSqToCenter(targetNodePos) < NODE_COMPLETION_DISTANCE * NODE_COMPLETION_DISTANCE) {
                path.advance();
                // The main completion check is at the beginning of this method
            }
        }

        // Method to smoothly release all movement keys
        private void releaseAllKeysSmoothly() {
            KeyBinding.setKeyBindState(mc.gameSettings.keyBindForward.getKeyCode(), false);
            KeyBinding.setKeyBindState(mc.gameSettings.keyBindBack.getKeyCode(), false);
            KeyBinding.setKeyBindState(mc.gameSettings.keyBindLeft.getKeyCode(), false);
            KeyBinding.setKeyBindState(mc.gameSettings.keyBindRight.getKeyCode(), false);
            KeyBinding.setKeyBindState(mc.gameSettings.keyBindJump.getKeyCode(), false);
            KeyBinding.setKeyBindState(mc.gameSettings.keyBindSprint.getKeyCode(), false);
            // Additional logic could be added here for a more gradual slowdown if needed
        }

        private boolean isStuckOnWall(EntityLivingBase entity) {
            BlockPos pos = new BlockPos(entity.posX, entity.posY, entity.posZ);
            BlockPos inFront = pos.offset(entity.getHorizontalFacing());
            return world.getBlockState(inFront).getBlock().isNormalCube() &&
                   !world.getBlockState(inFront.up()).getBlock().isNormalCube();
        }

        private boolean isSurroundedBySolidBlocks(EntityLivingBase entity) {
            BlockPos pos = new BlockPos(entity.posX, entity.posY, entity.posZ);
            // Check immediate surrounding blocks at head and feet level
            for (int dy = 0; dy <= 1; dy++) { // Check at two vertical levels
                for (EnumFacing facing : EnumFacing.HORIZONTALS) {
                    BlockPos checkPos = pos.offset(facing).add(0, dy, 0);
                    if (!world.getBlockState(checkPos).getBlock().isNormalCube()) {
                        return false; // Found an opening
                    }
                }
            }
            return true; // Surrounded by solid blocks
        }

        private boolean shouldJump(EntityLivingBase entity, PathNode node) {
            if (!entity.onGround) return false;

            // Check if we need to jump up to the next node based on the path action
            if (node.action == ActionType.JUMP) return true;

            BlockPos pos = new BlockPos(entity.posX, entity.posY, entity.posZ);
            BlockPos inFront = pos.offset(entity.getHorizontalFacing());
            Block blockInFront = world.getBlockState(inFront).getBlock();

            // **Refined logic:** Check if there is a walkable block directly in front at the same level
            // and there is space to jump on top of it.
            if (environmentScanner.isWalkable(inFront)) {
                 BlockPos blockAboveInFront = inFront.up();
                 if (!world.getBlockState(blockAboveInFront).getBlock().isNormalCube()) {
                      // If the block in front is something we typically jump onto (solid, stair, slab)
                      // and there's space above it, trigger a jump.
                      if (blockInFront.isNormalCube() || blockInFront instanceof BlockStairs || blockInFront instanceof BlockSlab) {
                           // Additionally, ensure the path is generally moving forward to prevent unnecessary jumps
                           Vec3 playerToNextNode = new Vec3(node.pos.getX() + 0.5 - entity.posX, node.pos.getY() + 0.5 - entity.posY, node.pos.getZ() + 0.5 - entity.posZ);
                           Vec3 playerDirection = entity.getLookVec();
                            // Check if the next node is roughly in front of the player
                           if (playerToNextNode.dotProduct(playerDirection) > 0.5) { // Use dot product to check if direction is similar
                                return true;
                           }
                      }
                 }
            }

            return false;
        }

        private boolean canSprint(EntityLivingBase entity, Path path) {
            // Removed player-specific checks like isBlocking() and isUsingItem()
            if (!entity.onGround || entity.isInWater() || entity.isSneaking()) return false;

            // Check if we're on a slime block or honey block (prevents sprinting)
            BlockPos pos = new BlockPos(entity.posX, entity.posY - 0.1, entity.posZ);
            Block groundBlock = world.getBlockState(pos).getBlock();
            if (groundBlock instanceof BlockSlime || groundBlock.getMaterial() == Material.clay) { // Clay is also slow
                return false;
            }

            int idx = path.currentIndex;
            if (idx >= path.nodes.size() - 1) return false;

            // Check the *next* node primarily, and one more ahead for turns/height changes
            PathNode currentNode = path.nodes.get(idx);
            PathNode nextNode = path.nodes.get(Math.min(idx + 1, path.nodes.size() - 1));

            // Don't sprint if the next node is a jump
            if (nextNode.action == ActionType.JUMP) return false;

            // Don't sprint if there's a significant height change to the next node
            if (Math.abs(nextNode.pos.getY() - currentNode.pos.getY()) > 1) return false;

            // Check for sharp turns ahead (look at the node after next if available)
            if (idx + 2 < path.nodes.size()) {
                PathNode nodeAfterNext = path.nodes.get(idx + 2);
                 // Check if the direction from current to next is significantly different from next to nodeAfterNext
                 EnumFacing dir1 = getDirectionBetween(currentNode.pos, nextNode.pos);
                 EnumFacing dir2 = getDirectionBetween(nextNode.pos, nodeAfterNext.pos);
                 if (dir1 != dir2 && dir1.getOpposite() != dir2) { // Not a straight line and not a simple U-turn
                     return false;
                 }
            }

            // We can sprint if the above checks pass, assuming we are facing the right way (handled by followPath)
            return true;
        }
    }

    private class EnvironmentScanner {
        private final World world;

        EnvironmentScanner(World world) {
            this.world = world;
        }

        public boolean isWalkable(BlockPos pos) {
            Block block = world.getBlockState(pos).getBlock();

            // Check if the block itself is passable
            if (block.isNormalCube() && !(block instanceof BlockStairs) && !(block instanceof BlockSlab)) {
                return false; // Solid block, can't walk through
            }

            // Check if there's solid ground below (for non-liquid blocks)
            BlockPos below = pos.down();
            Block belowBlock = world.getBlockState(below).getBlock();

            // Allow walking on various solid surfaces
            if (belowBlock.isNormalCube() ||
                    belowBlock instanceof BlockStairs ||
                    belowBlock instanceof BlockSlab ||
                    belowBlock.getMaterial() == Material.grass ||
                    belowBlock.getMaterial() == Material.ground) {

                // Check head clearance (2 blocks high for player)
                BlockPos head1 = pos.up();
                BlockPos head2 = pos.up(2);

                Block headBlock1 = world.getBlockState(head1).getBlock();
                Block headBlock2 = world.getBlockState(head2).getBlock();

                return !headBlock1.isNormalCube() && !headBlock2.isNormalCube();
            }

            // Allow walking in water/liquid if there's solid ground below
            if (block.getMaterial().isLiquid()) {
                return belowBlock.isNormalCube();
            }

            return false;
        }

        public boolean isSolid(BlockPos pos) {
            Block block = world.getBlockState(pos).getBlock();
             // A block is solid if it's a full normal cube
             return block.isNormalCube() && !(block instanceof BlockStairs) && !(block instanceof BlockSlab); // Exclude stairs and slabs as fully solid
        }

        public boolean canJumpUpTo(BlockPos from, BlockPos to) {
             // Check if there is space above the 'to' block for the player to stand
             BlockPos aboveTo = to.up();
             return !world.getBlockState(aboveTo).getBlock().isNormalCube();
        }

        public boolean hasLineOfSight(BlockPos start, BlockPos end) {
            return world.rayTraceBlocks(
                    new Vec3(start.getX() + 0.5, start.getY() + 1.0, start.getZ() + 0.5),
                    new Vec3(end.getX() + 0.5, end.getY() + 1.0, end.getZ() + 0.5),
                    false, true, false
            ) == null;
        }

        public boolean isChunkLoaded(BlockPos pos) {
            return world.getChunkProvider().chunkExists(pos.getX() >> 4, pos.getZ() >> 4);
        }

        public boolean isDangerous(BlockPos pos) {
            Block block = world.getBlockState(pos).getBlock();
            Material mat = block.getMaterial();
            return mat == Material.lava || mat == Material.fire;
        }

        public boolean isLiquid(BlockPos pos) {
            return world.getBlockState(pos).getBlock().getMaterial().isLiquid();
        }

        public boolean isFlatArea(BlockPos start, BlockPos end) {
            return Math.abs(end.getY() - start.getY()) < 2;
        }

        public boolean hasObstacleBetween(BlockPos start, BlockPos end) {
            // Check for obstacles using a ray trace and expanding the check around the line
            double startX = start.getX() + 0.5;
            double startY = start.getY() + 0.5; // Start check from block center
            double startZ = start.getZ() + 0.5;

            double endX = end.getX() + 0.5;
            double endY = end.getY() + 0.5; // End check at block center
            double endZ = end.getZ() + 0.5;

            Vec3 startVec = new Vec3(startX, startY, startZ);
            Vec3 endVec = new Vec3(endX, endY, endZ);

            // Perform a ray trace first for a quick check (checking line between block centers)
            MovingObjectPosition hit = world.rayTraceBlocks(startVec, endVec, false, true, false);

            if (hit != null && hit.typeOfHit == MovingObjectPosition.MovingObjectType.BLOCK) {
                Block hitBlock = world.getBlockState(hit.getBlockPos()).getBlock();
                // Consider it an obstacle if it blocks movement
                if (hitBlock.isNormalCube() && !(hitBlock instanceof BlockStairs) && !(hitBlock instanceof BlockSlab) && !(hitBlock instanceof BlockLadder) && !(hitBlock instanceof BlockVine)) {
                    return true;
                }
            }

            // Now, perform a more detailed volumetric check along the path for obstacles within the player's bounding box
            double playerWidth = 0.3; // Approximate half-width of a player
            double playerHeight = 1.8; // Approximate height of a player
            int steps = (int) startVec.distanceTo(endVec) * 8; // Increased check granularity
            if (steps == 0) steps = 1; // Ensure at least one step for very short segments

            for (int i = 0; i <= steps; i++) {
                double t = (double) i / steps;
                double currentX = startX + (endX - startX) * t;
                double currentY = startY + (endY - startY) * t;
                double currentZ = startZ + (endZ - startZ) * t;

                // Define a bounding box around the current point at player height
                BlockPos minPos = new BlockPos(currentX - playerWidth, currentY, currentZ - playerWidth); // Check from current Y
                BlockPos maxPos = new BlockPos(currentX + playerWidth, currentY + playerHeight - 0.1, currentZ + playerWidth); // Check up to player height

                for (int x = minPos.getX(); x <= maxPos.getX(); x++) {
                    for (int y = minPos.getY(); y <= maxPos.getY(); y++) {
                        for (int z = minPos.getZ(); z <= maxPos.getZ(); z++) {
                            BlockPos checkPos = new BlockPos(x, y, z);
                            Block block = world.getBlockState(checkPos).getBlock();

                            // Consider it an obstacle if it's a block that impedes movement
                             if (block.isNormalCube() && !(block instanceof BlockStairs) && !(block instanceof BlockSlab) && !(block instanceof BlockLadder) && !(block instanceof BlockVine)) {
                                return true; // Found an obstacle within the player's path volume
                            }
                        }
                    }
                }
            }

            return false; // No significant obstacle found
        }

        // New method to check for valid stair steps
        public boolean isValidStairStep(BlockPos from, BlockPos to) {
            // A valid stair step moves one block up and one block forward/sideways
            int dx = Math.abs(to.getX() - from.getX());
            int dy = to.getY() - from.getY(); // dy should be 1 for an upward step
            int dz = Math.abs(to.getZ() - from.getZ());

            if (dy != 1) return false; // Must be moving exactly one block up
            if (dx > 1 || dz > 1 || (dx == 0 && dz == 0)) return false; // Must move one block horizontally

            Block toBlock = world.getBlockState(to).getBlock();
            if (!(toBlock instanceof BlockStairs)) return false; // The target block must be a stair

            // Basic check: ensure the block below the target stair is solid
            BlockPos belowTo = to.down();
            if (!isSolid(belowTo)) return false;

            // More advanced check would involve checking the stair's orientation and shape
            // For now, we'll assume any move one block up and one horizontal onto a stair is valid

            return true;
        }
    }

    private enum ActionType {
        WALK, JUMP, FALL, SPRINT
    }

    // ----------------------------
    //       Public API
    // ----------------------------

    /** Enable/disable console debug prints. */
    public void setDebugEnabled(boolean debugEnabled) {
        this.debugEnabled = debugEnabled;
    }

    /**
     * Returns true if either:
     *  - A valid path was found and you have walked it all the way through, OR
     *  - No path was found in the last A* attempt (so there's nothing to walk).
     */
    public boolean isPathComplete() {
        if (currentPath == null && noPathFound) {
            return true;
        }
        if (currentPath == null) {
            // Either still calculating, or not yet requested—treat as "complete" so tick handler won't try to follow
            return true;
        }
        return currentPath.isComplete();
    }

    /** Cancel any existing path immediately. */
    public void cancelPath() {
        currentPath = null;
        noPathFound = false;
    }

    /** Adjust the movement cost for a specific block type. */
    public void setBlockCost(Block block, double cost) {
        blockCosts.put(block, cost);
    }

    /** Shut down the A* executor thread (call on mod unload). */
    public void shutdown() {
        executor.shutdown();
    }

    private boolean isPathAccessible(Path path) {
        if (path == null || path.nodes.isEmpty()) return false;

        for (int i = 0; i < path.nodes.size() - 1; i++) {
            PathNode current = path.nodes.get(i);
            PathNode next = path.nodes.get(i + 1);

            // Check if we can actually move between these nodes
            if (!canMoveBetween(current.pos, next.pos)) {
                return false;
            }

            // Check surrounding blocks for accessibility
            if (!isNodeAccessible(next.pos)) {
                return false;
            }
        }

        return true;
    }

    private boolean canMoveBetween(BlockPos from, BlockPos to) {
        // Check if there's a clear path between the nodes
        if (!environmentScanner.hasLineOfSight(from, to)) {
            return false;
        }

        // Check if the height difference is manageable
        int heightDiff = Math.abs(to.getY() - from.getY());
        if (heightDiff > 2) {
            return false;
        }

        // Check if we can actually walk to the next position
        if (!environmentScanner.isWalkable(to) || !environmentScanner.isWalkable(from)) {
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
            if (environmentScanner.isDangerous(checkPos)) {
                return false;
            }
            
            // Check for solid blocks that might block movement
            Block block = world.getBlockState(checkPos).getBlock();
            if (block.isNormalCube() && !(block instanceof BlockStairs) && !(block instanceof BlockSlab)) {
                return false;
            }
        }

        return environmentScanner.isWalkable(to) && 
               environmentScanner.isWalkable(from) &&
               !environmentScanner.isDangerous(to);
    }

    private boolean isNodeAccessible(BlockPos pos) {
        // Check surrounding blocks for accessibility with wider range
        for (int dx = -ACCESSIBILITY_CHECK_RANGE; dx <= ACCESSIBILITY_CHECK_RANGE; dx++) {
            for (int dz = -ACCESSIBILITY_CHECK_RANGE; dz <= ACCESSIBILITY_CHECK_RANGE; dz++) {
                BlockPos checkPos = pos.add(dx, 0, dz);
                
                // Check for dangerous blocks
                if (environmentScanner.isDangerous(checkPos)) {
                    return false;
                }
                
                // Check for solid blocks that might block movement
                if (Math.abs(dx) <= 2 && Math.abs(dz) <= 2) {
                    Block block = world.getBlockState(checkPos).getBlock();
                    if (block.isNormalCube() && !(block instanceof BlockStairs) && !(block instanceof BlockSlab)) {
                        return false;
                    }
                }
            }
        }
        return true;
    }

    private Path selectBestPath(List<Path> paths) {
        if (paths.isEmpty()) return null;

        Path bestPath = paths.get(0);
        double bestScore = Double.MAX_VALUE;

        for (Path path : paths) {
            double score = evaluatePath(path);
            if (score < bestScore) {
                bestScore = score;
                bestPath = path;
            }
        }

        return bestPath;
    }

    private double evaluatePath(Path path) {
        if (path == null || path.nodes.isEmpty()) {
            return Double.MAX_VALUE; // Invalid path
        }

        // Prioritize the path with the lowest total movement cost
        // The gCost of the last node represents the accumulated cost from the start
        PathNode lastNode = path.nodes.get(path.nodes.size() - 1);
        return lastNode.gCost;
    }

    private EnumFacing getDirectionBetween(BlockPos from, BlockPos to) {
        int dx = to.getX() - from.getX();
        int dz = to.getZ() - from.getZ();
        
        if (Math.abs(dx) > Math.abs(dz)) {
            return dx > 0 ? EnumFacing.EAST : EnumFacing.WEST;
        } else {
            return dz > 0 ? EnumFacing.SOUTH : EnumFacing.NORTH;
        }
    }

    private PathNode findClosestAccessibleNode(Collection<PathNode> nodes, BlockPos goal) {
        PathNode closest = null;
        double minDist = Double.MAX_VALUE;
        
        for (PathNode node : nodes) {
            if (!isNodeAccessible(node.pos)) continue;
            
            double dist = node.pos.distanceSq(goal);
            if (dist < minDist) {
                minDist = dist;
                closest = node;
            }
        }
        
        return closest;
    }

    private int countObstaclesAround(BlockPos pos) {
        int obstacleCount = 0;
        
        // Check in a wider area for obstacles
        for (int dx = -OBSTACLE_CHECK_RANGE; dx <= OBSTACLE_CHECK_RANGE; dx++) {
            for (int dz = -OBSTACLE_CHECK_RANGE; dz <= OBSTACLE_CHECK_RANGE; dz++) {
                for (int dy = -OBSTACLE_CHECK_HEIGHT; dy <= OBSTACLE_CHECK_HEIGHT; dy++) {
                    BlockPos checkPos = pos.add(dx, dy, dz);
                    
                    // Skip the position itself and positions too far vertically
                    if (dx == 0 && dz == 0 && dy == 0) continue;
                    
                    Block block = world.getBlockState(checkPos).getBlock();
                    
                    // Count solid blocks as obstacles
                    if (block.isNormalCube()) {
                        // Weight obstacles based on distance
                        double distance = Math.sqrt(dx * dx + dz * dz);
                        if (distance <= 2) {
                            obstacleCount += 3; // Close obstacles are more important
                        } else if (distance <= 3) {
                            obstacleCount += 2;
                        } else {
                            obstacleCount += 1;
                        }
                    }
                    
                    // Count dangerous blocks
                    if (environmentScanner.isDangerous(checkPos)) {
                        obstacleCount += 4; // Dangerous blocks are very bad
                    }
                }
            }
        }
        
        return obstacleCount;
    }
}
