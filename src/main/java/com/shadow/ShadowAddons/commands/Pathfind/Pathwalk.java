package com.shadow.ShadowAddons.commands.Pathfind;

import com.shadow.ShadowAddons.ShadowAddons;
import com.shadow.ShadowAddons.utils.CommandUtils.Command;
import com.shadow.ShadowAddons.utils.pathfinding.PathfindingIntegration;
import com.shadow.ShadowAddons.utils.pathfinding.PathfindingAPI;
import com.shadow.ShadowAddons.utils.pathfinding.SAStarPathfinder;
import com.shadow.ShadowAddons.utils.RotationsUtils.RotationHandler;
import net.minecraft.client.Minecraft;
import net.minecraft.client.entity.EntityPlayerSP;
import net.minecraft.command.CommandException;
import net.minecraft.command.ICommandSender;
import net.minecraft.util.BlockPos;
import net.minecraft.util.ChatComponentText;
import net.minecraft.util.MathHelper;
import net.minecraft.util.Vec3;
import net.minecraft.world.World;
import net.minecraftforge.fml.common.eventhandler.SubscribeEvent;
import net.minecraftforge.fml.common.gameevent.TickEvent;
import net.minecraftforge.common.MinecraftForge;

import java.awt.Toolkit;
import java.awt.datatransfer.Clipboard;
import java.awt.datatransfer.StringSelection;
import java.util.List;
import java.util.Map;
import java.util.HashMap;

public class Pathwalk {
    private static boolean isWalking = false;

    // Timeout / tick variables
    private static long lastUpdateTime = 0;
    private static final long UPDATE_INTERVAL = 50;   // ~20 ticks/sec
    private static final long RECALC_INTERVAL = 5000; // 5 sec
    private static long lastRecalcTime = 0;

    // How long we allow any single A* search (ms)
    private static final long MAX_SEARCH_TIME_MS = 5000;
    private static long pathSearchStartTime = 0;

    // Current A* path
    private static List<BlockPos> currentPath = null;
    private static int currentPathIndex = 0;
    public static BlockPos currentTarget = null;
    private static int currentRadius = SAStarPathfinder.Config.MAX_SEARCH_RADIUS;

    // Waypoints
    private static Map<String, BlockPos> waypoints = new HashMap<>();

    // Movement state tracking
    private static float targetYaw = 0;
    private static float targetPitch = 0;
    private static boolean hasSetInitialRotation = false;
    private static long lastRotationUpdate = 0;
    private static final long ROTATION_UPDATE_INTERVAL = 100; // Update rotation every 100ms
    private static final float ROTATION_SPEED = 15.0f; // Degrees per update
    private static final float YAW_TOLERANCE = 50.0f; // Degrees - how aligned we need to be to move

    public static void loadCommands() {
        ShadowAddons.commandManager.addCommand(new Command("pathwalk") {
            @Override
            public void processCommand(ICommandSender sender, String[] args) throws CommandException {
                if (args.length < 1) {
                    sender.addChatMessage(new ChatComponentText("§e=== Pathwalk Commands ==="));
                    sender.addChatMessage(new ChatComponentText("§a/pathwalk <x> <y> <z> §7- Walk to coordinates"));
                    sender.addChatMessage(new ChatComponentText("§a/pathwalk <x> <y> <z> <radius> §7- Walk with custom radius"));
                    sender.addChatMessage(new ChatComponentText("§a/pathwalk here §7- Display current position"));
                    sender.addChatMessage(new ChatComponentText("§a/pathwalk waypoint <name> §7- Save current position"));
                    sender.addChatMessage(new ChatComponentText("§a/pathwalk goto <name> §7- Walk to saved waypoint"));
                    return;
                }

                EntityPlayerSP player = Minecraft.getMinecraft().thePlayer;
                if (player == null) {
                    sender.addChatMessage(new ChatComponentText("§cPlayer not found"));
                    return;
                }

                try {
                    // "here" sub‐command
                    if (args[0].equalsIgnoreCase("here")) {
                        BlockPos ppos = new BlockPos(player.posX, player.posY, player.posZ);
                        sender.addChatMessage(
                                new ChatComponentText("§aCurrent position: " +
                                        ppos.getX() + " " + ppos.getY() + " " + ppos.getZ()));
                        return;
                    }

                    // "waypoint" sub‐command
                    if (args[0].equalsIgnoreCase("waypoint") && args.length >= 2) {
                        saveWaypoint(args[1], player);
                        sender.addChatMessage(
                                new ChatComponentText("§aWaypoint '" + args[1] + "' saved"));
                        return;
                    }

                    // "goto" sub‐command
                    if (args[0].equalsIgnoreCase("goto") && args.length >= 2) {
                        initializePathfinderIfNeeded(player.getEntityWorld());
                        BlockPos wp = getWaypoint(args[1]);
                        if (wp != null) {
                            startPathfinding(wp, -1, sender);
                            sender.addChatMessage(
                                    new ChatComponentText("§aStarted walking to waypoint '" + args[1] + "'"));
                        } else {
                            sender.addChatMessage(
                                    new ChatComponentText("§cWaypoint '" + args[1] + "' not found"));
                        }
                        return;
                    }

                    // Parse raw coordinates
                    if (args.length < 3) {
                        sender.addChatMessage(
                                new ChatComponentText("§cNeed at least 3 coordinates (x y z)"));
                        return;
                    }

                    initializePathfinderIfNeeded(player.getEntityWorld());

                    double x = Double.parseDouble(args[0]);
                    double y = Double.parseDouble(args[1]);
                    double z = Double.parseDouble(args[2]);
                    BlockPos target = new BlockPos(x, y, z);

                    // Optional custom radius
                    int customRadius = -1;
                    if (args.length >= 4) {
                        customRadius = Integer.parseInt(args[3]);
                        if (customRadius < 10 || customRadius > 500) {
                            sender.addChatMessage(
                                    new ChatComponentText("§cRadius must be between 10 and 500"));
                            return;
                        }
                    }

                    // Already at target?
                    BlockPos ppos = new BlockPos(player.posX, player.posY, player.posZ);
                    if (ppos.equals(target)) {
                        sender.addChatMessage(
                                new ChatComponentText("§a[Pathfinder] You are already at the target."));
                        return;
                    }

                    // If already walking, stop
                    if (isWalking) {
                        sender.addChatMessage(
                                new ChatComponentText("§eAlready walking. Stopping current path..."));
                        stopPathfinding();
                    }

                    // Compute straight‐line distance
                    double dist = Math.sqrt(ppos.distanceSq(target));

                    // Determine the actual radius to use:
                    if (customRadius > 0) {
                        currentRadius = customRadius;
                    } else {
                        int buffer = 10;
                        int desired = (int) Math.ceil(dist) + buffer;
                        currentRadius = Math.min(desired, SAStarPathfinder.Config.MAX_SEARCH_RADIUS);
                    }

                    // Warn if distance > radius
                    if (dist > currentRadius) {
                        sender.addChatMessage(new ChatComponentText(
                                "§eDistance (" + String.format("%.1f", dist) +
                                        ") exceeds radius (" + currentRadius +
                                        "). Path may fail."));
                    }

                    startPathfinding(target, currentRadius, sender);
                    sender.addChatMessage(new ChatComponentText(
                            "§aStarted pathfinding to " + target +
                                    " (Distance: " + String.format("%.1f", dist) + " blocks, Radius: " +
                                    currentRadius + ")"));
                }
                catch (NumberFormatException e) {
                    sender.addChatMessage(new ChatComponentText("§cInvalid coordinates or radius"));
                }
                catch (Exception e) {
                    sender.addChatMessage(new ChatComponentText("§cError: " + e.getMessage()));
                    e.printStackTrace();
                }
            }
        });

        ShadowAddons.commandManager.addCommand(new Command("stopwalk") {
            @Override
            public void processCommand(ICommandSender sender, String[] args) throws CommandException {
                if (isWalking) {
                    stopPathfinding();
                    sender.addChatMessage(new ChatComponentText("§aStopped pathfinding"));
                }
                else {
                    sender.addChatMessage(new ChatComponentText("§cNot currently pathfinding"));
                }
            }
        });

        ShadowAddons.commandManager.addCommand(new Command("pathdebug") {
            @Override
            public void processCommand(ICommandSender sender, String[] args) throws CommandException {
                if (ShadowAddons.pathfinder == null) {
                    sender.addChatMessage(new ChatComponentText("§cPathfinder not initialized yet."));
                    return;
                }
                if (args.length >= 1) {
                    String option = args[0].toLowerCase();
                    switch (option) {
                        case "path":
                            PathfindingAPI.toggleDebug("path");
                            sender.addChatMessage(new ChatComponentText("§aToggled path visualization"));
                            break;
                        case "nodes":
                            PathfindingAPI.toggleDebug("nodes");
                            sender.addChatMessage(new ChatComponentText("§aToggled node visualization"));
                            break;
                        case "costs":
                            PathfindingAPI.toggleDebug("costs");
                            sender.addChatMessage(new ChatComponentText("§aToggled cost display"));
                            break;
                        case "metrics":
                            PathfindingAPI.toggleDebug("metrics");
                            sender.addChatMessage(new ChatComponentText("§aToggled performance metrics"));
                            break;
                        case "clear":
                            PathfindingAPI.clearCache();
                            sender.addChatMessage(new ChatComponentText("§aCleared pathfinding cache"));
                            break;
                        default:
                            sender.addChatMessage(new ChatComponentText("§e/pathdebug [path|nodes|costs|metrics|clear]"));
                    }
                }
                else {
                    boolean newDebugState = !ShadowAddons.pathfinder.isDebugEnabled();
                    ShadowAddons.pathfinder.setDebugEnabled(newDebugState);
                    sender.addChatMessage(new ChatComponentText("§aPath debugging " + (newDebugState ? "enabled" : "disabled")));
                }
            }
        });

        ShadowAddons.commandManager.addCommand(new Command("copypath") {
            @Override
            public void processCommand(ICommandSender sender, String[] args) throws CommandException {
                EntityPlayerSP player = Minecraft.getMinecraft().thePlayer;
                if (player == null) return;

                if (args.length >= 1 && args[0].equalsIgnoreCase("target") && currentTarget != null) {
                    String coords = String.format("/pathwalk %d %d %d",
                            currentTarget.getX(), currentTarget.getY(), currentTarget.getZ());
                    copyToClipboard(coords);
                    player.addChatMessage(new ChatComponentText("§a[Path] Target copied: " + coords));
                }
                else {
                    int x = (int) Math.floor(player.posX);
                    int y = (int) Math.floor(player.posY);
                    int z = (int) Math.floor(player.posZ);
                    String coords = String.format("/pathwalk %d %d %d", x, y, z);
                    copyToClipboard(coords);
                    player.addChatMessage(new ChatComponentText("§a[Path] Current coords copied: " + coords));
                }
            }
        });

        ShadowAddons.commandManager.addCommand(new Command("pathstatus") {
            @Override
            public void processCommand(ICommandSender sender, String[] args) throws CommandException {
                if (!isWalking) {
                    sender.addChatMessage(new ChatComponentText("§cNot currently pathfinding"));
                    return;
                }
                if (ShadowAddons.pathfinder == null) {
                    sender.addChatMessage(new ChatComponentText("§cPathfinder not initialized"));
                    return;
                }

                EntityPlayerSP player = Minecraft.getMinecraft().thePlayer;
                if (player == null) return;

                BlockPos ppos = new BlockPos(player.posX, player.posY, player.posZ);
                double distToTarget = currentTarget != null
                        ? Math.sqrt(ppos.distanceSq(currentTarget)) : -1;

                sender.addChatMessage(new ChatComponentText("§e=== Pathfinding Status ==="));
                if (currentTarget != null) {
                    sender.addChatMessage(new ChatComponentText("§aTarget: " + currentTarget +
                            " (" + String.format("%.1f", distToTarget) + " blocks)"));
                }
                if (ShadowAddons.pathfinder.isCalculating()) {
                    sender.addChatMessage(new ChatComponentText("§eStatus: Calculating path..."));
                }
                else if (ShadowAddons.pathfinder.hasNoPath()) {
                    sender.addChatMessage(new ChatComponentText("§cStatus: No path found"));
                }
                else {
                    sender.addChatMessage(new ChatComponentText("§aStatus: Following path"));
                    if (currentPath != null) {
                        sender.addChatMessage(new ChatComponentText("§aPath length: " + currentPath.size() + " nodes"));
                        sender.addChatMessage(new ChatComponentText("§aCurrent node: " + (currentPathIndex + 1) + "/" + currentPath.size()));
                    }
                }

                SAStarPathfinder.PathResult lastResult = ShadowAddons.pathfinder.getLastResult();
                if (lastResult != null) {
                    sender.addChatMessage(new ChatComponentText("§aLast search: " +
                            lastResult.searchTime + "ms, " + lastResult.nodesExplored + " nodes"));
                }
            }
        });

        MinecraftForge.EVENT_BUS.register(new Object() {
            @SubscribeEvent
            public void onTick(TickEvent.ClientTickEvent event) {
                if (event.phase == TickEvent.Phase.END && isWalking) {
                    try {
                        updatePathfinding();
                    }
                    catch (Exception e) {
                        System.err.println("Error in pathfinding update: " + e.getMessage());
                        e.printStackTrace();
                        stopPathfinding();
                    }
                }
            }
        });
    }

    /**
     * Make sure SAStar is initialized once, and create our PathfindingIntegration.
     */
    private static void initializePathfinderIfNeeded(World world) {
        try {
            SAStarPathfinder.getInstance();
        } catch (IllegalStateException e) {
            try {
                SAStarPathfinder.initialize(world);
            } catch (Exception inner) {
                System.err.println("Failed to initialize SAStarPathfinder: " + inner.getMessage());
            }
        }
        if (ShadowAddons.pathfinder == null) {
            ShadowAddons.pathfinder = new PathfindingIntegration();
        }
    }

    /**
     * Start a single A* search (no chunking). We clamp radius to
     * (straight‐line distance + 10), never exceeding config max.
     */
    private static void startPathfinding(BlockPos target, int requestedRadius, ICommandSender sender) {
        EntityPlayerSP player = Minecraft.getMinecraft().thePlayer;
        if (player == null) return;

        // Reset any existing path
        stopPathfinding();

        // Figure out a "smart" radius:
        BlockPos start = new BlockPos(player.posX, player.posY, player.posZ);
        double dist = Math.sqrt(start.distanceSq(target));
        if (requestedRadius > 0) {
            currentRadius = requestedRadius;
        } else {
            int buffer = 10;
            int desired = (int) Math.ceil(dist) + buffer;
            currentRadius = Math.min(desired, SAStarPathfinder.Config.MAX_SEARCH_RADIUS);
        }

        if (dist > currentRadius) {
            sender.addChatMessage(new ChatComponentText(
                    "§eWarning: straight‐line distance (" +
                            String.format("%.1f", dist) + ") exceeds chosen radius (" +
                            currentRadius + "). A* may fail."));
        }

        // Remember the target & start the search
        currentTarget = target;
        pathSearchStartTime = System.currentTimeMillis();
        ShadowAddons.pathfinder.findPathAsync(start, target, currentRadius);

        // Reset movement state
        hasSetInitialRotation = false;
        targetYaw = 0;
        targetPitch = 0;
        lastRotationUpdate = 0;

        isWalking = true;
        lastUpdateTime = System.currentTimeMillis();
        lastRecalcTime = System.currentTimeMillis();
    }


    private static void updatePathfinding() {
        if (!isWalking || ShadowAddons.pathfinder == null) return;

        long now = System.currentTimeMillis();
        if (now - lastUpdateTime < UPDATE_INTERVAL) return;
        lastUpdateTime = now;

        EntityPlayerSP player = Minecraft.getMinecraft().thePlayer;
        if (player == null) {
            stopPathfinding();
            return;
        }

        // 1) If A* is still calculating and exceeded our timeout → abort
        if (ShadowAddons.pathfinder.isCalculating()
                && now - pathSearchStartTime > MAX_SEARCH_TIME_MS) {
            stopPathfinding();
            player.addChatMessage(new ChatComponentText(
                    "§c[Pathfinder] Aborting: search timed out."));
            return;
        }

        // 2) If A* just finished (and found a path), grab it immediately
        if (!ShadowAddons.pathfinder.isCalculating() && !ShadowAddons.pathfinder.hasNoPath()) {
            List<BlockPos> path = ShadowAddons.pathfinder.getCurrentPath();
            if (path != null && !path.equals(currentPath)) {
                currentPath = path;
                currentPathIndex = 0;
                hasSetInitialRotation = false; // Reset rotation when new path is set
            }
        }

        // 3) If no path was found, abort
        if (!ShadowAddons.pathfinder.isCalculating() && ShadowAddons.pathfinder.hasNoPath()) {
            stopPathfinding();
            player.addChatMessage(new ChatComponentText("§c[Pathfinder] No path found."));
            return;
        }

        // 4) If we have a valid path, follow it
        if (currentPath != null && currentPathIndex < currentPath.size()) {
            BlockPos nextNode = currentPath.get(currentPathIndex);
            BlockPos currentPos = new BlockPos(player.posX, player.posY, player.posZ);

            // Check if we've reached the current node
            double dx = (nextNode.getX() + 0.5) - player.posX;
            double dy = nextNode.getY() - player.posY;
            double dz = (nextNode.getZ() + 0.5) - player.posZ;
            double distSq = dx * dx + dy * dy + dz * dz;

            // If within completion threshold, advance to next node
            if (distSq < 0.8 * 0.8) {  // Within 0.8 blocks in 3D space
                currentPathIndex++;
                hasSetInitialRotation = false; // Reset rotation for new node

                if (currentPathIndex >= currentPath.size()) {
                    // Reached final destination
                    stopPathfinding();
                    player.addChatMessage(new ChatComponentText(
                            "§a[Pathfinder] Reached destination!"));
                    return;
                }
                nextNode = currentPath.get(currentPathIndex);
            }

            // Also check if we're close to the final target
            if (currentTarget != null) {
                double targetDx = (currentTarget.getX() + 0.5) - player.posX;
                double targetDy = currentTarget.getY() - player.posY;
                double targetDz = (currentTarget.getZ() + 0.5) - player.posZ;
                double targetDistSq = targetDx * targetDx + targetDy * targetDy + targetDz * targetDz;

                if (targetDistSq < 1.2 * 1.2) { // Within 1.2 blocks of final target
                    stopPathfinding();
                    player.addChatMessage(new ChatComponentText(
                            "§a[Pathfinder] Reached destination!"));
                    return;
                }
            }

            followCurrentPath(player, nextNode);
        }

        // 5) Periodically recalculate path if needed
        if (now - lastRecalcTime > RECALC_INTERVAL) {
            lastRecalcTime = now;
            BlockPos currentPos = new BlockPos(player.posX, player.posY, player.posZ);
            if (ShadowAddons.pathfinder != null && currentTarget != null) {
                ShadowAddons.pathfinder.recalculateIfNeeded(currentPos, currentTarget, currentRadius);
            }
        }
    }


private static void followCurrentPath(EntityPlayerSP player, BlockPos nextNode) {
    // Calculate direction to target
    double dx = (nextNode.getX() + 0.5) - player.posX;
    double dz = (nextNode.getZ() + 0.5) - player.posZ;
    double horizDist = Math.sqrt(dx * dx + dz * dz);

    // Improved stuck detection: check for both horizontal collision and lack of progress
    boolean isStuck = player.isCollidedHorizontally;
    if (!isStuck) {
        // Raycast to check for immediate obstructions
        isStuck = isBlockObstructing(player);
    }

    if (isStuck) {
        // Recalculate path if stuck for more than 1 second
        if (System.currentTimeMillis() - lastProgressTime > 1000) {
            BlockPos currentPos = new BlockPos(player.posX, player.posY, player.posZ);
            if (currentTarget != null) {
                // Request path recalculation
                ShadowAddons.pathfinder.recalculateIfNeeded(currentPos, currentTarget, currentRadius);
                lastProgressTime = System.currentTimeMillis(); // Reset timer
                return; // Skip movement this tick while recalculating
            }
        }
    } else {
        // Update progress time when not stuck
        lastProgressTime = System.currentTimeMillis();
    }

    // Calculate required yaw for target direction
    double targetYaw = Math.toDegrees(Math.atan2(dz, dx)) - 90.0;

    // Get current yaw in range -180 to 180
    float currentYaw = MathHelper.wrapAngleTo180_float(player.rotationYaw);

    // Get target yaw in same range
    targetYaw = MathHelper.wrapAngleTo180_double(targetYaw);

    // Calculate shortest rotation path
    float yawDiff = (float) MathHelper.wrapAngleTo180_double(targetYaw - currentYaw);

    // Smooth rotation with variable speed based on angle difference
    float baseSpeed = 2.0f;
    float maxSpeed = 8.0f;
    float rotationSpeed = Math.min(Math.max(Math.abs(yawDiff) * 0.3f, baseSpeed), maxSpeed);

    // Apply rotation if difference is noticeable
    if (Math.abs(yawDiff) > 1.0f) {
        // Use sign of yawDiff to determine direction and apply smooth rotation
        float rotation = Math.min(rotationSpeed, Math.abs(yawDiff)) * Math.signum(yawDiff);
        player.rotationYaw += rotation;
    }

    // Keep pitch level
    player.rotationPitch = 0;

    // Release all movement keys first
    releaseMovementKeys();

    // Only move if we're roughly facing the right direction (within 25 degrees)
    if (Math.abs(yawDiff) < 25.0f) {
        if (horizDist > 0.1) {
            Minecraft mc = Minecraft.getMinecraft();

            // Always move forward when there's distance
            net.minecraft.client.settings.KeyBinding.setKeyBindState(
                    mc.gameSettings.keyBindForward.getKeyCode(), true);

            // Sprint behavior - humans sprint when they have clear distance
            if (horizDist > 3.0) {
                net.minecraft.client.settings.KeyBinding.setKeyBindState(
                        mc.gameSettings.keyBindSprint.getKeyCode(), true);
            }

            // Smarter strafing - help correct course when slightly off angle
            if (Math.abs(yawDiff) > 5.0f && horizDist > 1.0) {
                if (yawDiff > 0) {
                    net.minecraft.client.settings.KeyBinding.setKeyBindState(
                            mc.gameSettings.keyBindRight.getKeyCode(), true);
                } else {
                    net.minecraft.client.settings.KeyBinding.setKeyBindState(
                            mc.gameSettings.keyBindLeft.getKeyCode(), true);
                }
            }
        }

        // Intelligent jumping - only when necessary
        double dy = nextNode.getY() - player.posY;
        if (player.onGround && dy > 0.5) {
            Minecraft mc = Minecraft.getMinecraft();
            net.minecraft.client.settings.KeyBinding.setKeyBindState(
                    mc.gameSettings.keyBindJump.getKeyCode(), true);
        }

        // Predictive jumping for blocks ahead
        if (currentPath != null && currentPathIndex + 1 < currentPath.size()) {
            BlockPos nextNextNode = currentPath.get(currentPathIndex + 1);
            double nextDy = nextNextNode.getY() - nextNode.getY();
            if (player.onGround && nextDy > 0.5 && horizDist < 2.0) {
                Minecraft mc = Minecraft.getMinecraft();
                net.minecraft.client.settings.KeyBinding.setKeyBindState(
                        mc.gameSettings.keyBindJump.getKeyCode(), true);
            }
        }
    }
}
private static long lastProgressTime = System.currentTimeMillis();


//create isBlockObstructing method to check for obstructions
private static boolean isBlockObstructing(EntityPlayerSP player) {
    World world = player.getEntityWorld();
    Vec3 playerPos = new Vec3(player.posX, player.posY + player.getEyeHeight(), player.posZ);
    Vec3 lookVec = player.getLookVec().normalize().addVector(1.5, 1.5, 1.5);
    Vec3 targetPos = playerPos.add(lookVec);

    // Check if there's a solid block in the way
    BlockPos targetBlockPos = new BlockPos(targetPos.xCoord, targetPos.yCoord, targetPos.zCoord);
    return world.getBlockState(targetBlockPos).getBlock().isFullBlock();
}


    private static void stopPathfinding() {
        isWalking = false;
        currentPath = null;
        currentPathIndex = 0;
        currentTarget = null;
        hasSetInitialRotation = false;
        targetYaw = 0;
        targetPitch = 0;

        // Stop external rotation handler if it exists
        RotationHandler.forceStop();

        if (ShadowAddons.pathfinder != null) {
            ShadowAddons.pathfinder.cancelPath();
        }
        releaseMovementKeys();
    }



    private static void releaseMovementKeys() {
        try {
            EntityPlayerSP player = Minecraft.getMinecraft().thePlayer;
            if (player == null) return;
            Minecraft mc = Minecraft.getMinecraft();
            net.minecraft.client.settings.KeyBinding.setKeyBindState(
                    mc.gameSettings.keyBindForward.getKeyCode(), false);
            net.minecraft.client.settings.KeyBinding.setKeyBindState(
                    mc.gameSettings.keyBindBack.getKeyCode(), false);
            net.minecraft.client.settings.KeyBinding.setKeyBindState(
                    mc.gameSettings.keyBindLeft.getKeyCode(), false);
            net.minecraft.client.settings.KeyBinding.setKeyBindState(
                    mc.gameSettings.keyBindRight.getKeyCode(), false);
            net.minecraft.client.settings.KeyBinding.setKeyBindState(
                    mc.gameSettings.keyBindJump.getKeyCode(), false);
            net.minecraft.client.settings.KeyBinding.setKeyBindState(
                    mc.gameSettings.keyBindSprint.getKeyCode(), false);
        } catch (Exception e) {
            System.err.println("Error releasing movement keys: " + e.getMessage());
        }
    }

    private static void saveWaypoint(String name, EntityPlayerSP player) {
        BlockPos pos = new BlockPos(player.posX, player.posY, player.posZ);
        waypoints.put(name.toLowerCase(), pos);
    }

    private static BlockPos getWaypoint(String name) {
        return waypoints.get(name.toLowerCase());
    }

    private static void copyToClipboard(String text) {
        try {
            StringSelection sel = new StringSelection(text);
            Clipboard cb = Toolkit.getDefaultToolkit().getSystemClipboard();
            cb.setContents(sel, null);
        } catch (Exception e) {
            System.err.println("Failed to copy to clipboard: " + e.getMessage());
        }
    }

    // Public getters
    public static boolean isCurrentlyWalking() {
        return isWalking;
    }

    public static Map<String, BlockPos> getWaypoints() {
        return new HashMap<>(waypoints);
    }

    public static boolean removeWaypoint(String name) {
        return waypoints.remove(name.toLowerCase()) != null;
    }

    public static void addWaypoint(String name, BlockPos pos) {
        waypoints.put(name.toLowerCase(), pos);
    }
}