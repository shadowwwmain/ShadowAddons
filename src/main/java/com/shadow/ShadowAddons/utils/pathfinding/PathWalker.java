package com.shadow.ShadowAddons.utils.pathfinding;

import net.minecraft.client.Minecraft;
import net.minecraft.client.settings.KeyBinding;
import net.minecraft.entity.player.EntityPlayer;
import net.minecraft.util.BlockPos;
import net.minecraft.util.MovingObjectPosition;
import net.minecraft.util.Vec3;
import net.minecraftforge.common.MinecraftForge;
import net.minecraftforge.fml.common.eventhandler.SubscribeEvent;
import net.minecraftforge.fml.common.gameevent.TickEvent;

import java.util.List;

public class PathWalker {

    private static PathWalker instance;
    private final Minecraft mc = Minecraft.getMinecraft();
    private final SAStarPathfinder pathfinder;

    // Walking state
    private List<BlockPos> currentPath;
    private int currentWaypointIndex;
    private BlockPos targetPos;
    private boolean isWalking;
    private boolean isPaused;
    private long lastStuckCheck;
    private BlockPos lastPlayerPos;
    private int stuckCounter;

    // Smooth rotation state
    private float targetYaw;
    private float targetPitch;
    private boolean hasRotationTarget = false;
    private long lastRotationUpdate = 0;

    // Configuration
    public static class Config {
        public static double MOVEMENT_SPEED = 0.3;
        public static double ROTATION_SPEED = 3.0; // Reduced for smoother rotation
        public static double WAYPOINT_REACH_DISTANCE = 1.2; // Reduced for more precise movement
        public static double TARGET_REACH_DISTANCE = 1.8; // Separate distance for final target
        public static double STUCK_THRESHOLD = 0.3; // More sensitive stuck detection
        public static long STUCK_CHECK_INTERVAL = 800; // ms
        public static int MAX_STUCK_COUNT = 4;
        public static boolean AUTO_JUMP = true;
        public static boolean ENABLE_SPRINT = false; // Disabled by default for better control
        public static boolean SMOOTH_ROTATION = true;
        public static boolean RECALCULATE_ON_STUCK = true;
        public static double LOOK_AHEAD_DISTANCE = 2.5;
        public static double ROTATION_THRESHOLD = 5.0; // Degrees - don't rotate for small angles
        public static double STAIR_JUMP_THRESHOLD = 0.6; // Height difference to trigger stair jumping
    }

    // Walking status
    public enum WalkingStatus {
        IDLE, WALKING, PAUSED, STUCK, FAILED, COMPLETED
    }

    private WalkingStatus status = WalkingStatus.IDLE;
    private String statusMessage = "";

    public PathWalker() {
        this.pathfinder = SAStarPathfinder.getInstance();
        MinecraftForge.EVENT_BUS.register(this);
        instance = this;
    }

    public static PathWalker getInstance() {
        if (instance == null) {
            new PathWalker();
        }
        return instance;
    }


    public static void intliaizePathfinder() {
        if (SAStarPathfinder.getInstance() == null) {
            SAStarPathfinder.initialize(Minecraft.getMinecraft().theWorld);
        }
    }

    // Main walking methods
    public boolean startWalking(BlockPos target) {
        if (mc.thePlayer == null) return false;

        intliaizePathfinder();

        this.targetPos = target;
        BlockPos playerPos = mc.thePlayer.getPosition();

        // Find path
        SAStarPathfinder.PathResult result = pathfinder.findPath(playerPos, target);

        if (!result.success) {
            status = WalkingStatus.FAILED;
            statusMessage = result.failureReason != null ? result.failureReason : "Pathfinding failed";
            return false;
        }

        this.currentPath = result.path;
        this.currentWaypointIndex = 0;
        this.isWalking = true;
        this.isPaused = false;
        this.status = WalkingStatus.WALKING;
        this.statusMessage = "Walking to target";
        this.lastStuckCheck = System.currentTimeMillis();
        this.lastPlayerPos = playerPos;
        this.stuckCounter = 0;
        this.hasRotationTarget = false;

        return true;
    }

    public void stopWalking() {
        isWalking = false;
        isPaused = false;
        currentPath = null;
        currentWaypointIndex = 0;
        status = WalkingStatus.IDLE;
        statusMessage = "Stopped";
        hasRotationTarget = false;

        // Stop all movement keys
        releaseAllKeys();
    }

    public void pauseWalking() {
        isPaused = true;
        status = WalkingStatus.PAUSED;
        statusMessage = "Paused";
        releaseAllKeys();
    }

    public void resumeWalking() {
        if (isWalking) {
            isPaused = false;
            status = WalkingStatus.WALKING;
            statusMessage = "Resumed walking";
        }
    }

    private void releaseAllKeys() {
        if (mc.thePlayer != null) {
            KeyBinding.setKeyBindState(mc.gameSettings.keyBindForward.getKeyCode(), false);
            KeyBinding.setKeyBindState(mc.gameSettings.keyBindBack.getKeyCode(), false);
            KeyBinding.setKeyBindState(mc.gameSettings.keyBindLeft.getKeyCode(), false);
            KeyBinding.setKeyBindState(mc.gameSettings.keyBindRight.getKeyCode(), false);
            KeyBinding.setKeyBindState(mc.gameSettings.keyBindJump.getKeyCode(), false);
            KeyBinding.setKeyBindState(mc.gameSettings.keyBindSprint.getKeyCode(), false);
        }
    }

    public boolean recalculatePath() {
        if (targetPos == null || mc.thePlayer == null) return false;

        BlockPos playerPos = mc.thePlayer.getPosition();
        SAStarPathfinder.PathResult result = pathfinder.findPath(playerPos, targetPos);

        if (result.success) {
            currentPath = result.path;
            currentWaypointIndex = 0;
            stuckCounter = 0;
            status = WalkingStatus.WALKING;
            statusMessage = "Path recalculated";
            hasRotationTarget = false;
            return true;
        } else {
            status = WalkingStatus.FAILED;
            statusMessage = "Failed to recalculate path";
            return false;
        }
    }

    @SubscribeEvent
    public void onClientTick(TickEvent.ClientTickEvent event) {
        if (event.phase != TickEvent.Phase.START || mc.thePlayer == null || !isWalking || isPaused) {
            return;
        }

        if (currentPath == null || currentPath.isEmpty()) {
            stopWalking();
            return;
        }

        // Check if we've reached the final target with better precision
        Vec3 playerPos = mc.thePlayer.getPositionVector();
        Vec3 targetVec = new Vec3(targetPos.getX() + 0.5, targetPos.getY(), targetPos.getZ() + 0.5);
        double distanceToFinalTarget = playerPos.distanceTo(targetVec);

        if (distanceToFinalTarget <= Config.TARGET_REACH_DISTANCE) {
            status = WalkingStatus.COMPLETED;
            statusMessage = "Reached target";
            stopWalking();
            return;
        }

        // Check for stuck detection
        checkStuckStatus();

        // Get current waypoint
        if (currentWaypointIndex >= currentPath.size()) {
            stopWalking();
            return;
        }

        BlockPos currentWaypoint = currentPath.get(currentWaypointIndex);

        // Check if we've reached current waypoint
        Vec3 waypointVec = new Vec3(currentWaypoint.getX() + 0.5, currentWaypoint.getY(), currentWaypoint.getZ() + 0.5);
        double distanceToWaypoint = playerPos.distanceTo(waypointVec);

        if (distanceToWaypoint <= Config.WAYPOINT_REACH_DISTANCE) {
            currentWaypointIndex++;
            if (currentWaypointIndex >= currentPath.size()) {
                // Don't stop here - let the final target check handle completion
                return;
            }
            currentWaypoint = currentPath.get(currentWaypointIndex);
            hasRotationTarget = false; // Reset rotation target for new waypoint
        }

        // Move towards waypoint
        moveTowards(currentWaypoint);
    }

    private void moveTowards(BlockPos target) {
        EntityPlayer player = mc.thePlayer;
        Vec3 playerPos = player.getPositionVector();
        Vec3 targetVec = new Vec3(target.getX() + 0.5, target.getY(), target.getZ() + 0.5);

        // Calculate direction (only horizontal for movement)
        Vec3 horizontalDirection = new Vec3(
                targetVec.xCoord - playerPos.xCoord,
                0,
                targetVec.zCoord - playerPos.zCoord
        ).normalize();

        // Smooth rotation
        if (Config.SMOOTH_ROTATION) {
            smoothRotateTowards(horizontalDirection);
        } else {
            rotateTowards(horizontalDirection);
        }

        // Check for obstacles and stairs
        boolean shouldJump = shouldJump(target);

        // Apply movement with better control
        applyControlledMovement(horizontalDirection, shouldJump);
    }

    private void rotateTowards(Vec3 direction) {
        float yaw = (float) (Math.atan2(-direction.xCoord, direction.zCoord) * 180.0 / Math.PI);
        mc.thePlayer.rotationYaw = yaw;
        // Don't adjust pitch for horizontal movement
    }

    private void smoothRotateTowards(Vec3 direction) {
        float newTargetYaw = (float) (Math.atan2(-direction.xCoord, direction.zCoord) * 180.0 / Math.PI);
        newTargetYaw = wrapAngleTo180(newTargetYaw);

        // Check if we need to update rotation target
        if (!hasRotationTarget || Math.abs(wrapAngleTo180(newTargetYaw - targetYaw)) > Config.ROTATION_THRESHOLD) {
            targetYaw = newTargetYaw;
            targetPitch = 0; // Keep pitch level for walking
            hasRotationTarget = true;
            lastRotationUpdate = System.currentTimeMillis();
        }

        // Calculate current rotation difference
        float currentYaw = wrapAngleTo180(mc.thePlayer.rotationYaw);
        float yawDiff = wrapAngleTo180(targetYaw - currentYaw);

        // Only rotate if the difference is significant
        if (Math.abs(yawDiff) > 1.0f) {
            // Smooth rotation with adaptive speed
            float rotationSpeed = (float) Math.max(1.0, Math.min(Config.ROTATION_SPEED, Math.abs(yawDiff) * 0.3));
            float yawStep = Math.min(Math.abs(yawDiff), rotationSpeed) * Math.signum(yawDiff);

            mc.thePlayer.rotationYaw = wrapAngleTo180(currentYaw + yawStep);
        }

        // Keep pitch stable for walking
        if (Math.abs(mc.thePlayer.rotationPitch) > 5.0f) {
            mc.thePlayer.rotationPitch *= 0.8f; // Gradually return to level
        }
    }

    private float wrapAngleTo180(float angle) {
        angle = angle % 360.0f;
        if (angle >= 180.0f) {
            angle -= 360.0f;
        } else if (angle < -180.0f) {
            angle += 360.0f;
        }
        return angle;
    }

    private void applyControlledMovement(Vec3 direction, boolean shouldJump) {
        // Calculate how aligned we are with the target direction
        Vec3 lookVector = getLookVector();
        double alignment = direction.dotProduct(lookVector);

        // Only move forward if we're reasonably aligned (prevents erratic movement)
        boolean moveForward = alignment > 0.7; // Require good alignment before moving

        // Calculate strafe movement for fine adjustments
        Vec3 strafeVector = getStrafeVector();
        double strafe = direction.dotProduct(strafeVector);

        // Apply movement with better control
        KeyBinding.setKeyBindState(mc.gameSettings.keyBindForward.getKeyCode(), moveForward);
        KeyBinding.setKeyBindState(mc.gameSettings.keyBindBack.getKeyCode(), false); // Never walk backwards

        // Use strafe for minor corrections only
        KeyBinding.setKeyBindState(mc.gameSettings.keyBindLeft.getKeyCode(), Math.abs(strafe) > 0.3 && strafe < -0.1);
        KeyBinding.setKeyBindState(mc.gameSettings.keyBindRight.getKeyCode(), Math.abs(strafe) > 0.3 && strafe > 0.1);

        // Improved jumping logic
        KeyBinding.setKeyBindState(mc.gameSettings.keyBindJump.getKeyCode(), Config.AUTO_JUMP && shouldJump);

        // Controlled sprinting (only when moving forward and aligned)
        boolean shouldSprint = Config.ENABLE_SPRINT && moveForward && alignment > 0.9;
        KeyBinding.setKeyBindState(mc.gameSettings.keyBindSprint.getKeyCode(), shouldSprint);
    }

    private Vec3 getLookVector() {
        float yaw = mc.thePlayer.rotationYaw * (float) Math.PI / 180.0f;
        return new Vec3(-Math.sin(yaw), 0, Math.cos(yaw));
    }

    private Vec3 getStrafeVector() {
        float yaw = (mc.thePlayer.rotationYaw + 90) * (float) Math.PI / 180.0f;
        return new Vec3(-Math.sin(yaw), 0, Math.cos(yaw));
    }

    private boolean shouldJump(BlockPos target) {
        EntityPlayer player = mc.thePlayer;
        BlockPos playerPos = player.getPosition();

        // Enhanced stair detection
        double heightDifference = target.getY() - playerPos.getY();

        // Jump for stairs (height difference between 0.5 and 1.5 blocks)
        if (heightDifference >= Config.STAIR_JUMP_THRESHOLD && heightDifference <= 1.5) {
            return true;
        }

        // Check for blocks directly in front that require jumping
        Vec3 playerPosVec = player.getPositionVector();
        Vec3 lookVector = getLookVector();
        Vec3 checkPos = playerPosVec.addVector(lookVector.xCoord * 1.2, 0, lookVector.zCoord * 1.2);

        BlockPos blockToCheck = new BlockPos(checkPos);

        // Check if there's a solid block at player height that we need to jump over
        if (!mc.theWorld.isAirBlock(blockToCheck) && mc.theWorld.isAirBlock(blockToCheck.up())) {
            return true;
        }

        // Check for general obstacles in path
        Vec3 start = player.getPositionEyes(1.0f);
        Vec3 end = new Vec3(target.getX() + 0.5, target.getY() + 1.0, target.getZ() + 0.5);

        MovingObjectPosition rayTrace = mc.theWorld.rayTraceBlocks(start, end, true, true, true);

        // Only jump if there's a block obstacle and we're not already jumping
        return rayTrace != null &&
                rayTrace.typeOfHit == MovingObjectPosition.MovingObjectType.BLOCK &&
                !player.isAirBorne;
    }

    private void checkStuckStatus() {
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastStuckCheck < Config.STUCK_CHECK_INTERVAL) {
            return;
        }

        BlockPos currentPos = mc.thePlayer.getPosition();

        if (lastPlayerPos != null) {
            double distance = currentPos.distanceSq(lastPlayerPos);

            if (distance < Config.STUCK_THRESHOLD * Config.STUCK_THRESHOLD) {
                stuckCounter++;

                if (stuckCounter >= Config.MAX_STUCK_COUNT) {
                    status = WalkingStatus.STUCK;
                    statusMessage = "Player appears to be stuck";

                    if (Config.RECALCULATE_ON_STUCK) {
                        if (!recalculatePath()) {
                            stopWalking();
                        }
                    }
                }
            } else {
                stuckCounter = 0;
                if (status == WalkingStatus.STUCK) {
                    status = WalkingStatus.WALKING;
                    statusMessage = "Walking resumed";
                }
            }
        }

        lastPlayerPos = currentPos;
        lastStuckCheck = currentTime;
    }

    // Getters (unchanged)
    public boolean isWalking() {
        return isWalking;
    }

    public boolean isPaused() {
        return isPaused;
    }

    public WalkingStatus getStatus() {
        return status;
    }

    public String getStatusMessage() {
        return statusMessage;
    }

    public List<BlockPos> getCurrentPath() {
        return currentPath;
    }

    public BlockPos getCurrentWaypoint() {
        if (currentPath == null || currentWaypointIndex >= currentPath.size()) {
            return null;
        }
        return currentPath.get(currentWaypointIndex);
    }

    public BlockPos getTargetPos() {
        return targetPos;
    }

    public double getDistanceToTarget() {
        if (targetPos == null || mc.thePlayer == null) {
            return -1;
        }
        return mc.thePlayer.getDistance(targetPos.getX(), targetPos.getY(), targetPos.getZ());
    }

    public int getWaypointProgress() {
        if (currentPath == null || currentPath.isEmpty()) {
            return 0;
        }
        return currentWaypointIndex;
    }

    public int getTotalWaypoints() {
        if (currentPath == null) {
            return 0;
        }
        return currentPath.size();
    }

    public double getPathCompletionPercentage() {
        if (currentPath == null || currentPath.isEmpty()) {
            return 0.0;
        }
        return (double) currentWaypointIndex / currentPath.size() * 100.0;
    }
}