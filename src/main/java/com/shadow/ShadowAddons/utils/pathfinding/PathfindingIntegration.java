package com.shadow.ShadowAddons.utils.pathfinding;

import com.shadow.ShadowAddons.utils.pathfinding.SAStarPathfinder;
import net.minecraft.client.Minecraft;
import net.minecraft.client.entity.EntityPlayerSP;
import net.minecraft.client.settings.KeyBinding;
import net.minecraft.util.BlockPos;
import net.minecraft.util.ChatComponentText;
import net.minecraft.util.EnumChatFormatting;
import net.minecraft.util.Vec3;
import net.minecraftforge.event.entity.player.PlayerEvent;
import scala.Int;

import java.util.List;

import static com.shadow.ShadowAddons.utils.MessageUtils.MessageUtil.sendColoredMessage;

public class PathfindingIntegration {
    private SAStarPathfinder.PathResult currentResult;
    private List<BlockPos> currentPath;
    private int currentPathIndex = 0;
    private static boolean isCalculating = false;
    private boolean hasNoPath = false;
    private boolean isPathComplete = false;
    private boolean debugEnabled = false;

    // Movement state
    private BlockPos lastPlayerPos;
    private long lastMovementTime = 0;
    private static final long STUCK_TIMEOUT = 3000; // 3 seconds

    public PathfindingIntegration() {
        // Initialize pathfinder if needed
        if (SAStarPathfinder.getInstance() == null) {
            PathfindingAPI.init(Minecraft.getMinecraft().theWorld);
        }
    }

    // Method to replace your old2.findPathAsync
    /**
     * Asynchronously finds a path from start to goal, using the given search radius.
     */
    public void findPathAsync(BlockPos start, BlockPos goal, int radius) {
        isCalculating = true;
        hasNoPath = false;
        isPathComplete = false;
        currentPathIndex = 0;

        // Initialize pathfinder if needed
        if (SAStarPathfinder.getInstance() == null) {
            PathfindingAPI.init(Minecraft.getMinecraft().theWorld);
        }

        // Run pathfinding in a separate thread to avoid blocking the main client
        new Thread(() -> {
            try {
                // Pass the radius into the PathfindingAPI call
                currentResult = PathfindingAPI.findPath(start, goal, radius);
                isCalculating = false;

                if (currentResult.success) {
                    currentPath = currentResult.path;
                    hasNoPath = false;
                    isPathComplete = false;

                    // Notify player of success
                    EntityPlayerSP player = Minecraft.getMinecraft().thePlayer;
                    if (player != null) {
                        player.addChatMessage(new ChatComponentText(
                                String.format("§a[Pathfinder] Path found! Length: %.1f blocks, Time: %dms",
                                        currentResult.pathLength, currentResult.searchTime)
                        ));
                    }
                } else {
                    hasNoPath = true;
                    currentPath = null;

                    // Notify player of failure reason
                    EntityPlayerSP player = Minecraft.getMinecraft().thePlayer;
                    if (player != null) {
                        player.addChatMessage(new ChatComponentText(
                                "§c[Pathfinder] " + currentResult.failureReason
                        ));
                    }
                }
            } catch (Exception e) {
                isCalculating = false;
                hasNoPath = true;
                e.printStackTrace();
            }
        }).start();
    }

    /**
     * Overload for backwards compatibility: uses the default radius from config.
     */
    public void findPathAsync(BlockPos start, BlockPos goal) {
        findPathAsync(start, goal, SAStarPathfinder.Config.MAX_SEARCH_RADIUS);
    }

    // Method to follow the calculated path
    public void followPath(EntityPlayerSP player) {
        if (currentPath == null || currentPath.isEmpty() || isPathComplete) return;

        BlockPos playerPos = new BlockPos(player.posX, player.posY, player.posZ);

        // Check if we're stuck
        if (lastPlayerPos != null && lastPlayerPos.equals(playerPos)) {
            if (System.currentTimeMillis() - lastMovementTime > STUCK_TIMEOUT) {
                // Player is stuck, try to recalculate or jump
                handleStuckPlayer(player);
                lastMovementTime = System.currentTimeMillis();
            }
        } else {
            lastPlayerPos = playerPos;
            lastMovementTime = System.currentTimeMillis();
        }

        // Find the next target position
        BlockPos targetPos = getNextPathTarget(playerPos);
        if (targetPos == null) {
            isPathComplete = true;
            return;
        }

        // Move towards target
        moveTowards(player, targetPos);
    }

    private BlockPos getNextPathTarget(BlockPos playerPos) {
        if (currentPath == null || currentPathIndex >= currentPath.size()) return null;

        // Skip to the farthest reachable point in the path
        for (int i = Math.min(currentPathIndex + 3, currentPath.size() - 1); i >= currentPathIndex; i--) {
            BlockPos pathPos = currentPath.get(i);
            if (playerPos.distanceSq(pathPos) < 9) { // Within 3 blocks
                currentPathIndex = i + 1;
                if (currentPathIndex >= currentPath.size()) {
                    return null; // Path complete
                }
                return currentPath.get(currentPathIndex);
            }
        }

        // Return current target if we haven't reached it yet
        return currentPath.get(currentPathIndex);
    }

    private void moveTowards(EntityPlayerSP player, BlockPos target) {
        Vec3 playerVec = new Vec3(player.posX, player.posY, player.posZ);
        Vec3 targetVec = new Vec3(target.getX() + 0.5, target.getY(), target.getZ() + 0.5);
        Vec3 direction = targetVec.subtract(playerVec).normalize();

        Minecraft mc = Minecraft.getMinecraft();

        // Calculate required movement
        double deltaX = direction.xCoord;
        double deltaZ = direction.zCoord;
        double deltaY = targetVec.yCoord - playerVec.yCoord;

        // Reset all movement keys
        setKeyState(mc.gameSettings.keyBindForward, false);
        setKeyState(mc.gameSettings.keyBindBack, false);
        setKeyState(mc.gameSettings.keyBindLeft, false);
        setKeyState(mc.gameSettings.keyBindRight, false);
        setKeyState(mc.gameSettings.keyBindJump, false);

        // Forward/backward movement
        if (Math.abs(deltaZ) > 0.1) {
            if (deltaZ > 0) {
                setKeyState(mc.gameSettings.keyBindForward, true);
            } else {
                setKeyState(mc.gameSettings.keyBindBack, true);
            }
        }

        // Left/right movement
        if (Math.abs(deltaX) > 0.1) {
            if (deltaX > 0) {
                setKeyState(mc.gameSettings.keyBindRight, true);
            } else {
                setKeyState(mc.gameSettings.keyBindLeft, true);
            }
        }

        // Jumping logic
        if (deltaY > 0.5 || needsJump(player, target)) {
            setKeyState(mc.gameSettings.keyBindJump, true);
        }

        // Sprint when moving forward
        if (deltaZ > 0.3) {
            setKeyState(mc.gameSettings.keyBindSprint, true);
        }
    }

    private boolean needsJump(EntityPlayerSP player, BlockPos target) {
        BlockPos playerPos = new BlockPos(player.posX, player.posY, player.posZ);

        // Check if there's a block in front that needs jumping over
        BlockPos front = playerPos.offset(player.getHorizontalFacing());
        if (!Minecraft.getMinecraft().theWorld.isAirBlock(front)) {
            return true;
        }

        // Check if we need to jump up
        return target.getY() > playerPos.getY();
    }

    private void handleStuckPlayer(EntityPlayerSP player) {
        // Try jumping to get unstuck
        setKeyState(Minecraft.getMinecraft().gameSettings.keyBindJump, true);

        // Recalculate path from current position
        BlockPos currentPos = new BlockPos(player.posX, player.posY, player.posZ);
        if (currentPath != null && !currentPath.isEmpty()) {
            BlockPos goal = currentPath.get(currentPath.size() - 1);
            findPathAsync(currentPos, goal);
        }
    }

    private void setKeyState(KeyBinding key, boolean state) {
        KeyBinding.setKeyBindState(key.getKeyCode(), state);
    }

    // Methods to match your old2 interface
    public static boolean isCalculating() {
        return isCalculating;
    }

    public boolean hasNoPath() {
        return hasNoPath;
    }

    public boolean isPathComplete() {
        return isPathComplete;
    }

    public void cancelPath() {
        currentPath = null;
        currentResult = null;
        isCalculating = false;
        hasNoPath = false;
        isPathComplete = false;
        currentPathIndex = 0;

        // Release all movement keys
        Minecraft mc = Minecraft.getMinecraft();
        setKeyState(mc.gameSettings.keyBindForward, false);
        setKeyState(mc.gameSettings.keyBindBack, false);
        setKeyState(mc.gameSettings.keyBindLeft, false);
        setKeyState(mc.gameSettings.keyBindRight, false);
        setKeyState(mc.gameSettings.keyBindJump, false);
        setKeyState(mc.gameSettings.keyBindSprint, false);
    }

    public void recalculateIfNeeded(BlockPos currentPos, BlockPos targetPos, int radius) {
        if (currentPath == null || hasNoPath || isCalculating) return;

        boolean needsRecalc = false;

        if (currentPathIndex < currentPath.size()) {
            BlockPos pathPos = currentPath.get(currentPathIndex);
            // Use squared distance for efficiency
            if (currentPos.distanceSq(pathPos) > (radius * radius)) {
                needsRecalc = true;
            }
        }
    }

    // Debug methods
    public void setDebugEnabled(boolean enabled) {
        debugEnabled = enabled;
        if (SAStarPathfinder.getInstance() != null) {
            PathfindingAPI.setDebugEnabled(enabled);
        }
    }

    public boolean isDebugEnabled() {
        return debugEnabled;
    }

    // Get current path for debugging
    public List<BlockPos> getCurrentPath() {
        return currentPath;
    }

    public SAStarPathfinder.PathResult getLastResult() {
        return currentResult;
    }
}
