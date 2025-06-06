package com.shadow.ShadowAddons.utils.pathfinding;

import net.minecraft.util.BlockPos;
import net.minecraft.world.World;

public class PathfindingAPI {

    /**
     * Initialize the pathfinding system
     */
    public static void init(World world) {
        SAStarPathfinder.initialize(world);
    }

    /**
     * Find a path between two positions
     */
    public static SAStarPathfinder.PathResult findPath(BlockPos start, BlockPos goal) {
        return SAStarPathfinder.getInstance().findPath(start, goal);
    }

    /**
     * Find a path with custom search radius
     */
    public static SAStarPathfinder.PathResult findPath(BlockPos start, BlockPos goal, int radius) {
        return SAStarPathfinder.getInstance().findPath(start, goal, radius);
    }

    /**
     * Get the current pathfinder instance
     */
    public static SAStarPathfinder getPathfinder() {
        return SAStarPathfinder.getInstance();
    }

    /**
     * Toggle debug visualization
     */
    public static void toggleDebug(String overlay) {
        SAStarPathfinder.getInstance().toggleDebugOverlay(overlay);
    }

    /**
     * Enable/disable debug rendering
     */
    public static void setDebugEnabled(boolean enabled) {
        SAStarPathfinder.getInstance().setDebugEnabled(enabled);
    }

    /**
     * Clear pathfinding cache
     */
    public static void clearCache() {
        SAStarPathfinder.getInstance().clearCache();
    }

    /**
     * Get last pathfinding result
     */
    public static SAStarPathfinder.PathResult getLastResult() {
        return SAStarPathfinder.getInstance().getLastResult();
    }
}