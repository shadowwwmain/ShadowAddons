package com.shadow.ShadowAddons.utils.pathfinding;

import net.minecraft.client.Minecraft;
import net.minecraft.client.entity.EntityPlayerSP;
import net.minecraft.client.settings.KeyBinding;
import net.minecraft.util.BlockPos;
import net.minecraft.util.Vec3;
import net.minecraft.util.MathHelper;
import net.minecraft.block.Block;
import java.util.List;
import java.util.ArrayList;
import net.minecraft.world.World;
import com.shadow.ShadowAddons.utils.pathfinding.SAStarPathfinder;

import static com.shadow.ShadowAddons.utils.pathfinding.SAStarPathfinder.PathNode.isPassable;
import static com.shadow.ShadowAddons.utils.pathfinding.SAStarPathfinder.handleObstacleAvoidance;

public class PathfindingIntegration {
    private SAStarPathfinder.PathResult currentResult;
    private List<BlockPos> currentPath;
    private int currentPathIndex = 0;
    private static boolean isCalculating = false;
    private boolean hasNoPath = false;
    private boolean isPathComplete = false;
    private boolean debugEnabled = false;

    private static final double REACHED_THRESHOLD = 1.5;
    private static final long STUCK_TIMEOUT = 3000;
    private BlockPos lastPlayerPos;
    private long lastMovementTime = 0;

    private boolean strafeLeft = false;
    private boolean strafeRight = false;
    private int stuckTicks = 0;
    private static final int STUCK_THRESHOLD = 40; // 2 seconds at 20 ticks/sec
    private BlockPos lastProgressPos = null;
    private long lastProgressTime = 0;
    private static final long PROGRESS_TIMEOUT = 3000; // 3 seconds

    // Movement capability tracking
    private double speedMultiplier = 1.0;
    private int jumpBoostLevel = 0;

    // Movement and rotation constants
    private static final float SMOOTH_ROTATION_SPEED = 8.0f; // Slower, smoother rotation
    private static final int WALL_LOOK_AHEAD = 4; // Look further ahead for walls
    private static final double WALL_CHECK_WIDTH = 2.0; // Check wider area for walls
    private static final int OBSTACLE_CHECK_RADIUS = 4; // Check larger radius around player

    public PathfindingIntegration() {
        if (SAStarPathfinder.getInstance() == null) {
            PathfindingAPI.init(Minecraft.getMinecraft().theWorld);
        }
    }

    public void updateMovementEffects(EntityPlayerSP player) {
        // Check for Speed effect
        net.minecraft.potion.PotionEffect speed = player.getActivePotionEffect(net.minecraft.potion.Potion.moveSpeed);
        if (speed != null) {
            speedMultiplier = 1.0 + ((speed.getAmplifier() + 1) * 0.2);
        } else {
            speedMultiplier = 1.0;
        }

        // Check for Jump Boost
        net.minecraft.potion.PotionEffect jump = player.getActivePotionEffect(net.minecraft.potion.Potion.jump);
        jumpBoostLevel = jump != null ? jump.getAmplifier() + 1 : 0;

        // Update pathfinder with new capabilities
        PathfindingAPI.updateMovementCapabilities(speedMultiplier, jumpBoostLevel);
    }

    public void findPathAsync(BlockPos start, BlockPos goal, int radius) {
        try {
            isCalculating = true;
            hasNoPath = false;
            isPathComplete = false;
            currentPathIndex = 0;

            if (SAStarPathfinder.getInstance() == null) {
                PathfindingAPI.init(Minecraft.getMinecraft().theWorld);
            }

            new Thread(() -> {
                try {
                    // For very long distances, use chunked pathfinding
                    double dist = Math.sqrt(start.distanceSq(goal));
                    if (dist > radius) {
                        currentResult = findLongDistancePath(start, goal);
                    } else {
                        currentResult = PathfindingAPI.findPath(start, goal, radius);
                    }
                    isCalculating = false;

                    EntityPlayerSP player = Minecraft.getMinecraft().thePlayer;
                    if (player == null) return;

                    if (currentResult.success) {
                        currentPath = currentResult.path;
                        hasNoPath = false;
                        notifySuccess(player);
                    } else {
                        hasNoPath = true;
                        currentPath = null;
                        notifyFailure(player);
                    }
                } catch (Exception e) {
                    isCalculating = false;
                    hasNoPath = true;
                    e.printStackTrace();
                }
            }).start();
        } catch (Exception e) {
            isCalculating = false;
            hasNoPath = true;
            e.printStackTrace();
        }
    }

    private SAStarPathfinder.PathResult findLongDistancePath(BlockPos start, BlockPos goal) {
        double totalDist = Math.sqrt(start.distanceSq(goal));
        int numChunks = (int) Math.ceil(totalDist / SAStarPathfinder.Config.CHUNK_SIZE);

        List<BlockPos> fullPath = new ArrayList<>();
        BlockPos current = start;

        for (int i = 1; i <= numChunks; i++) {
            double progress = i / (double) numChunks;
            BlockPos intermediateTarget = new BlockPos(
                start.getX() + (goal.getX() - start.getX()) * progress,
                start.getY() + (goal.getY() - start.getY()) * progress,
                start.getZ() + (goal.getZ() - start.getZ()) * progress
            );

            SAStarPathfinder.PathResult chunkResult = PathfindingAPI.findPath(
                current,
                intermediateTarget,
                SAStarPathfinder.Config.CHUNK_SIZE
            );

            if (!chunkResult.success) {
                return chunkResult; // Return the failed result
            }

            if (!fullPath.isEmpty() && !chunkResult.path.isEmpty()) {
                chunkResult.path.remove(0); // Remove duplicate point
            }
            fullPath.addAll(chunkResult.path);
            current = intermediateTarget;
        }

        return new SAStarPathfinder.PathResult(
            fullPath,
            true,
            0,
            fullPath.size(),
            totalDist,
            null
        );
    }

    private void notifySuccess(EntityPlayerSP player) {
        if (player != null && currentResult != null) {
            player.addChatMessage(new net.minecraft.util.ChatComponentText(
                String.format("§a[Pathfinder] Path found! Length: %.1f blocks, Time: %dms",
                    currentResult.pathLength, currentResult.searchTime)
            ));
        }
    }

    private void notifyFailure(EntityPlayerSP player) {
        if (player != null && currentResult != null) {
            player.addChatMessage(new net.minecraft.util.ChatComponentText(
                "§c[Pathfinder] " + currentResult.failureReason
            ));
        }
    }

    public void findPathAsync(BlockPos start, BlockPos goal) {
        findPathAsync(start, goal, SAStarPathfinder.Config.MAX_SEARCH_RADIUS);
    }

    public void followPath(EntityPlayerSP player) {
        if (currentPath == null || currentPath.isEmpty() || isPathComplete || player == null) return;

        BlockPos playerPos = new BlockPos(player.posX, player.posY, player.posZ);
        handleStuckDetection(player, playerPos);

        BlockPos target = getNextPathTarget(playerPos);
        if (target == null) {
            isPathComplete = true;
            releaseKeys();
            return;
        }

        moveTowards(player, target);
    }

    private void handleStuckDetection(EntityPlayerSP player, BlockPos playerPos) {
        if (lastPlayerPos != null && lastPlayerPos.equals(playerPos)) {
            stuckTicks++;

            // Check if we're making any progress towards target
            if (lastProgressPos == null ||
                !lastProgressPos.equals(playerPos)) {
                lastProgressPos = playerPos;
                lastProgressTime = System.currentTimeMillis();
            }

            // If stuck for too long or no progress, force recalculation
            if (stuckTicks >= STUCK_THRESHOLD ||
                System.currentTimeMillis() - lastProgressTime > PROGRESS_TIMEOUT) {
                if (currentPath != null && !currentPath.isEmpty()) {
                    BlockPos goal = currentPath.get(currentPath.size() - 1);
                    // Try to find alternative path around obstacle
                    findAlternativePath(player, goal);
                }
                stuckTicks = 0;
                lastProgressTime = System.currentTimeMillis();
            }
        } else {
            stuckTicks = 0;
            lastPlayerPos = playerPos;
            lastMovementTime = System.currentTimeMillis();
        }
    }

    private void findAlternativePath(EntityPlayerSP player, BlockPos goal) {
        // Get current position and look direction
        BlockPos playerPos = new BlockPos(player.posX, player.posY, player.posZ);
        Vec3 lookVec = player.getLookVec();

        // Try to find alternative paths by checking different offsets
        List<BlockPos> alternativeStarts = new ArrayList<>();

        // Check left and right of current position
        Vec3 right = new Vec3(-lookVec.zCoord, 0, lookVec.xCoord).normalize();
        for (int i = 2; i <= 4; i++) {
            // Right side
            alternativeStarts.add(playerPos.add(
                right.xCoord * i,
                0,
                right.zCoord * i
            ));
            // Left side
            alternativeStarts.add(playerPos.add(
                -right.xCoord * i,
                0,
                -right.zCoord * i
            ));
        }

        // Also try positions slightly behind
        Vec3 back = lookVec.normalize();
        alternativeStarts.add(playerPos.add(-back.xCoord * 3, 0, -back.zCoord * 3));

        // Try different Y levels
        for (int y = -1; y <= 1; y++) {
            if (y != 0) {
                alternativeStarts.add(playerPos.add(0, y, 0));
            }
        }

        // Try each alternative start point until we find a valid path
        for (BlockPos altStart : alternativeStarts) {
            if (isValidPosition(altStart)) {
                findPathAsync(altStart, goal);
                break;
            }
        }
    }

    private boolean isValidPosition(BlockPos pos) {
        World world = Minecraft.getMinecraft().theWorld;
        if (world == null) return false;

        // Check if position is safe to stand
        Block block = world.getBlockState(pos).getBlock();
        Block below = world.getBlockState(pos.down()).getBlock();
        Block above = world.getBlockState(pos.up()).getBlock();

        return below.isNormalCube() &&
               !block.isNormalCube() &&
               !above.isNormalCube();
    }

    private BlockPos getNextPathTarget(BlockPos playerPos) {
        if (currentPath == null || currentPathIndex >= currentPath.size()) return null;

        for (int i = Math.min(currentPathIndex + 2, currentPath.size() - 1); i >= currentPathIndex; i--) {
            BlockPos pathPos = currentPath.get(i);
            if (playerPos.distanceSq(pathPos) < REACHED_THRESHOLD * REACHED_THRESHOLD) {
                currentPathIndex = i + 1;
                return currentPathIndex < currentPath.size() ? currentPath.get(currentPathIndex) : null;
            }
        }

        return currentPath.get(currentPathIndex);
    }

    private boolean shouldJump(EntityPlayerSP player, BlockPos target) {
        if (jumpBoostLevel > 0) return true; // Always jump if we have jump boost

        // Check if the target is significantly higher than the player
        double heightDiff = target.getY() - player.posY;
        if (heightDiff > 0.5) return true; // Jump if target is more than 1 block higher

        // Check for obstacles in front of the player
        Vec3 look = player.getLookVec();

        // Check immediate blocks
        BlockPos checkPos = new BlockPos(
                player.posX + look.xCoord,
                player.posY,
                player.posZ + look.zCoord
        );

        // Check blocks one step ahead
        BlockPos checkAheadPos = new BlockPos(
                player.posX + look.xCoord * 2,
                player.posY,
                player.posZ + look.zCoord * 2
        );

        // Return true if there's any obstacle at feet or head level
        return !isPassable(checkPos) || !isPassable(checkPos.up()) ||
                !isPassable(checkAheadPos) || !isPassable(checkAheadPos.up());
    }


    private boolean isInsideWall(EntityPlayerSP player) {
        BlockPos playerPos = new BlockPos(player.posX, player.posY, player.posZ);
        World world = Minecraft.getMinecraft().theWorld;

        // Check if player's head or body is inside a block
        return !isPassable(playerPos) || !isPassable(playerPos.up());
    }

    private void handleStuckInWall(EntityPlayerSP player, BlockPos target) {
        // Try to find nearest open space
        BlockPos playerPos = new BlockPos(player.posX, player.posY, player.posZ);
        Vec3 toTarget = new Vec3(
            target.getX() - player.posX,
            target.getY() - player.posY,
            target.getZ() - player.posZ
        ).normalize();

        // Check in a spiral pattern for escape
        BlockPos bestEscape = null;
        double bestScore = Double.MAX_VALUE;

        for (int dist = 1; dist <= 3; dist++) {
            for (double angle = 0; angle < Math.PI * 2; angle += Math.PI / 4) {
                double x = Math.cos(angle) * dist;
                double z = Math.sin(angle) * dist;
                BlockPos check = new BlockPos(
                    playerPos.getX() + x,
                    playerPos.getY(),
                    playerPos.getZ() + z
                );

                if (isPassable(check) && isPassable(check.up()) && !isPassable(check.down())) {
                    // Score based on distance to target and clearance
                    double score = check.distanceSq(target) +
                                 (hasAnyObstacleNear(check) ? 50 : 0);
                    if (score < bestScore) {
                        bestScore = score;
                        bestEscape = check;
                    }
                }
            }
        }

        Minecraft mc = Minecraft.getMinecraft();
        if (bestEscape != null) {
            // Move towards escape point
            Vec3 toEscape = new Vec3(
                bestEscape.getX() + 0.5 - player.posX,
                bestEscape.getY() - player.posY,
                bestEscape.getZ() + 0.5 - player.posZ
            ).normalize();

            // Force rotation to escape direction
            float yaw = (float) Math.toDegrees(Math.atan2(-toEscape.xCoord, toEscape.zCoord));
            player.rotationYaw = yaw;

            // Sprint and jump to escape
            setKeyState(mc.gameSettings.keyBindSprint, true);
            setKeyState(mc.gameSettings.keyBindForward, true);
            setKeyState(mc.gameSettings.keyBindJump, true);
        } else {
            // Emergency escape - just jump and move back
            setKeyState(mc.gameSettings.keyBindBack, true);
            setKeyState(mc.gameSettings.keyBindJump, true);
        }
    }

    private void moveTowards(EntityPlayerSP player, BlockPos target) {
        updateMovementEffects(player);

        // First check if we're stuck in a wall
        if (isInsideWall(player)) {
            handleStuckInWall(player, target);
            return;
        }

        Vec3 movement = calculateMovement(player, target);
        boolean shouldJump = shouldJump(player, target);

        releaseKeys();

        // Handle obstacles first
        if (hasLargeObstacleAhead(player)) {
            handleObstacleAvoidance(player, target);
            return;
        }

        // Normal movement
        Minecraft mc = Minecraft.getMinecraft();
        double dist = Math.sqrt(player.getDistanceSq(target.getX() + 0.5, player.posY, target.getZ() + 0.5));

        // Smoother speed control
        boolean shouldSprint = dist > 3.0 * speedMultiplier;
        setKeyState(mc.gameSettings.keyBindSprint, shouldSprint);
        setKeyState(mc.gameSettings.keyBindForward, true);

        // Apply jump boost-aware jumping
        if (shouldJump) {
            setKeyState(mc.gameSettings.keyBindJump, true);
            if (jumpBoostLevel > 1) {
                player.rotationPitch = MathHelper.clamp_float(player.rotationPitch + 5.0f, -90.0f, 45.0f);
            }
        }
    }

    private Vec3 calculateMovement(EntityPlayerSP player, BlockPos target) {
        double dx = (target.getX() + 0.5) - player.posX;
        double dy = target.getY() - player.posY;
        double dz = (target.getZ() + 0.5) - player.posZ;

        // Calculate target rotation
        float targetYaw = (float) Math.toDegrees(Math.atan2(-dx, dz));
        float yawDifference = MathHelper.wrapAngleTo180_float(targetYaw - player.rotationYaw);

        // Apply smooth rotation
        float smoothYaw = yawDifference / SMOOTH_ROTATION_SPEED;
        player.rotationYaw = MathHelper.wrapAngleTo180_float(player.rotationYaw + smoothYaw);

        // Calculate pitch based on distance and height
        double horizontalDist = Math.sqrt(dx * dx + dz * dz);
        float targetPitch = (float) Math.toDegrees(-Math.atan2(dy, horizontalDist));
        float pitchDifference = targetPitch - player.rotationPitch;
        float smoothPitch = pitchDifference / SMOOTH_ROTATION_SPEED;
        player.rotationPitch = MathHelper.clamp_float(player.rotationPitch + smoothPitch, -90.0f, 90.0f);

        return new Vec3(dx, dy, dz).normalize();
    }

    private void applyMovement(Vec3 movement, boolean jump, boolean strafe) {
        Minecraft mc = Minecraft.getMinecraft();

        // Basic movement
        setKeyState(mc.gameSettings.keyBindForward, true);
        setKeyState(mc.gameSettings.keyBindSprint, true);

        // Apply strafing if needed
        if (strafe) {
            if (strafeRight) {
                setKeyState(mc.gameSettings.keyBindRight, true);
            } else if (strafeLeft) {
                setKeyState(mc.gameSettings.keyBindLeft, true);
            }
        }

        // Jump for obstacles or height differences
        setKeyState(mc.gameSettings.keyBindJump, jump);

        // If we're stuck, try to wiggle free
        if (isStuck()) {
            wiggleFree();
        }
    }

    private boolean isStuck() {
        EntityPlayerSP player = Minecraft.getMinecraft().thePlayer;
        if (player == null) return false;

        // Check if we haven't moved significantly in the last second
        return lastPlayerPos != null &&
               lastPlayerPos.equals(new BlockPos(player.posX, player.posY, player.posZ)) &&
               System.currentTimeMillis() - lastMovementTime > 1000;
    }

    private void wiggleFree() {
        EntityPlayerSP player = Minecraft.getMinecraft().thePlayer;
        if (player == null) return;

        // Try to move sideways briefly to get unstuck
        Minecraft mc = Minecraft.getMinecraft();
        if ((System.currentTimeMillis() / 500) % 2 == 0) {
            setKeyState(mc.gameSettings.keyBindLeft, true);
            setKeyState(mc.gameSettings.keyBindRight, false);
        } else {
            setKeyState(mc.gameSettings.keyBindLeft, false);
            setKeyState(mc.gameSettings.keyBindRight, true);
        }

        // Always try jumping when stuck
        setKeyState(mc.gameSettings.keyBindJump, true);
    }

    private void jumpAndRecalculate(EntityPlayerSP player) {
        setKeyState(Minecraft.getMinecraft().gameSettings.keyBindJump, true);
        if (currentPath != null && !currentPath.isEmpty()) {
            BlockPos goal = currentPath.get(currentPath.size() - 1);
            findPathAsync(new BlockPos(player.posX, player.posY, player.posZ), goal);
        }
        lastMovementTime = System.currentTimeMillis();
    }

    public void cancelPath() {
        currentPath = null;
        currentResult = null;
        isCalculating = false;
        hasNoPath = false;
        isPathComplete = false;
        currentPathIndex = 0;
        releaseKeys();
    }

    private void releaseKeys() {
        Minecraft mc = Minecraft.getMinecraft();
        setKeyState(mc.gameSettings.keyBindForward, false);
        setKeyState(mc.gameSettings.keyBindBack, false);
        setKeyState(mc.gameSettings.keyBindLeft, false);
        setKeyState(mc.gameSettings.keyBindRight, false);
        setKeyState(mc.gameSettings.keyBindJump, false);
        setKeyState(mc.gameSettings.keyBindSprint, false);
    }

    private void setKeyState(KeyBinding key, boolean state) {
        KeyBinding.setKeyBindState(key.getKeyCode(), state);
    }

    public void recalculateIfNeeded(BlockPos currentPos, BlockPos targetPos, int radius) {
        if (currentPath == null || hasNoPath || isCalculating) return;
        if (currentPathIndex < currentPath.size() &&
            currentPos.distanceSq(currentPath.get(currentPathIndex)) > (radius * radius)) {
            findPathAsync(currentPos, targetPos, radius);
        }
    }

    public static boolean isCalculating() { return isCalculating; }
    public boolean hasNoPath() { return hasNoPath; }
    public boolean isPathComplete() { return isPathComplete; }
    public void setDebugEnabled(boolean enabled) {
        debugEnabled = enabled;
        if (SAStarPathfinder.getInstance() != null) {
            PathfindingAPI.setDebugEnabled(enabled);
        }
    }
    public boolean isDebugEnabled() { return debugEnabled; }
    public List<BlockPos> getCurrentPath() { return currentPath; }
    public SAStarPathfinder.PathResult getLastResult() { return currentResult; }

    private boolean hasObstacleAhead(EntityPlayerSP player) {
        // Check a few blocks ahead in the look direction
        Vec3 look = player.getLookVec();
        for (int i = 1; i <= 2; i++) {
            BlockPos check = new BlockPos(
                player.posX + look.xCoord * i,
                player.posY,
                player.posZ + look.zCoord * i
            );

            // Check both at feet level and head level
            if (!isPassable(check) || !isPassable(check.up())) {
                return true;
            }

            // Also check for diagonal obstacles
            for (int dx = -1; dx <= 1; dx++) {
                for (int dz = -1; dz <= 1; dz++) {
                    if (dx == 0 && dz == 0) continue;
                    BlockPos diag = check.add(dx, 0, dz);
                    if (!isPassable(diag) || !isPassable(diag.up())) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    private boolean hasLargeObstacleAhead(EntityPlayerSP player) {
        Vec3 look = player.getLookVec();
        BlockPos playerPos = new BlockPos(player.posX, player.posY, player.posZ);

        // Check in a wide arc ahead for large obstacles
        for (int dist = 1; dist <= WALL_LOOK_AHEAD; dist++) {
            for (double offset = -WALL_CHECK_WIDTH; offset <= WALL_CHECK_WIDTH; offset += 0.5) {
                // Check wider as we get further out
                double scaledOffset = offset * (dist / 2.0);
                BlockPos checkPos = playerPos.add(
                    look.xCoord * dist + look.zCoord * scaledOffset,
                    0,
                    look.zCoord * dist - look.xCoord * scaledOffset
                );

                // Check for full-height obstacles
                if (isFullHeightObstacle(checkPos)) {
                    return true;
                }

                // Check for large obstacle patterns
                if (hasLargeObstaclePattern(checkPos)) {
                    return true;
                }
            }
        }
        return false;
    }

    private boolean isFullHeightObstacle(BlockPos pos) {
        World world = Minecraft.getMinecraft().theWorld;
        boolean hasObstacle = false;
        int obstacleHeight = 0;

        // Check for vertical obstacles
        for (int y = 0; y <= 2; y++) {
            BlockPos check = pos.add(0, y, 0);
            if (!isPassable(check)) {
                obstacleHeight++;
                hasObstacle = true;
            }
        }

        return hasObstacle && obstacleHeight >= 2;
    }

    private boolean hasLargeObstaclePattern(BlockPos center) {
        int obstacleCount = 0;

        // Check for 3x3 pattern of obstacles
        for (int dx = -1; dx <= 1; dx++) {
            for (int dz = -1; dz <= 1; dz++) {
                BlockPos check = center.add(dx, 0, dz);
                if (!isPassable(check)) {
                    obstacleCount++;
                }
            }
        }

        return obstacleCount >= 4; // If 4 or more blocks in 3x3 area are obstacles
    }

    private void handleObstacleAvoidance(EntityPlayerSP player, BlockPos target) {
        if (hasLargeObstacleAhead(player)) {
            // Find best avoidance direction
            Vec3 look = player.getLookVec();
            Vec3 right = new Vec3(-look.zCoord, 0, look.xCoord).normalize();
            BlockPos playerPos = new BlockPos(player.posX, player.posY, player.posZ);

            double rightClearance = checkDirectionClearance(playerPos, right, target);
            double leftClearance = checkDirectionClearance(playerPos, new Vec3(-right.xCoord, -right.yCoord, -right.zCoord), target);
            Minecraft mc = Minecraft.getMinecraft();
            if (rightClearance > leftClearance) {
                setKeyState(mc.gameSettings.keyBindRight, true);
                setKeyState(mc.gameSettings.keyBindLeft, false);
            } else {
                setKeyState(mc.gameSettings.keyBindLeft, true);
                setKeyState(mc.gameSettings.keyBindRight, false);
            }

            // Slow down near obstacles
            setKeyState(mc.gameSettings.keyBindSprint, false);
            setKeyState(mc.gameSettings.keyBindForward, true);
            setKeyState(mc.gameSettings.keyBindJump, true);
        }
    }

    private double checkDirectionClearance(BlockPos pos, Vec3 dir, BlockPos target) {
        double clearance = 0;
        double targetAlignment = 0;

        // Check increasing distances in this direction
        for (int dist = 1; dist <= OBSTACLE_CHECK_RADIUS; dist++) {
            BlockPos check = pos.add(
                dir.xCoord * dist,
                0,
                dir.zCoord * dist
            );

            if (!hasAnyObstacleNear(check)) {
                clearance += 1.0;
                // Bonus for positions closer to target
                targetAlignment += 1.0 / (1.0 + Math.sqrt(check.distanceSq(target)));
            } else {
                break;
            }
        }

        return clearance + (targetAlignment * 0.5);
    }

    private boolean hasAnyObstacleNear(BlockPos pos) {
        for (int dx = -1; dx <= 1; dx++) {
            for (int dz = -1; dz <= 1; dz++) {
                BlockPos check = pos.add(dx, 0, dz);
                if (!isPassable(check) || !isPassable(check.up())) {
                    return true;
                }
            }
        }
        return false;
    }
}
