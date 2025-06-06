package com.shadow.ShadowAddons.utils;

import net.minecraft.entity.EntityLivingBase;
import net.minecraft.util.MathHelper;
import net.minecraft.util.Vec3;

public class RotationUtils {
    public static class PathFindRotation {
        private static final float ROTATION_SPEED = 0.15f; // Reduced for smoother rotation
        private static float lastYaw = 0;
        private static float lastPitch = 0;

        public static void rotateToPosition(EntityLivingBase entity, Vec3 targetPos) {
            double dx = targetPos.xCoord - entity.posX;
            double dy = targetPos.yCoord - (entity.posY + entity.getEyeHeight());
            double dz = targetPos.zCoord - entity.posZ;
            
            // Calculate target angles
            float targetYaw = (float) Math.toDegrees(Math.atan2(dz, dx)) - 90.0f;
            float targetPitch = (float) -Math.toDegrees(Math.atan2(dy, Math.sqrt(dx * dx + dz * dz)));
            
            // Smooth rotation with interpolation
            float yawDiff = MathHelper.wrapAngleTo180_float(targetYaw - entity.rotationYaw);
            float pitchDiff = MathHelper.wrapAngleTo180_float(targetPitch - entity.rotationPitch);
            
            // Apply smooth rotation
            if (Math.abs(yawDiff) > 0.1f) {
                entity.rotationYaw += yawDiff * ROTATION_SPEED;
            }
            if (Math.abs(pitchDiff) > 0.1f) {
                entity.rotationPitch += pitchDiff * ROTATION_SPEED;
            }
            
            // Store last rotation
            lastYaw = entity.rotationYaw;
            lastPitch = entity.rotationPitch;
        }

        public static float getRotationDifference(float current, float target) {
            return MathHelper.wrapAngleTo180_float(target - current);
        }

        public static boolean isFacingPosition(EntityLivingBase entity, Vec3 targetPos, float maxAngle) {
            double dx = targetPos.xCoord - entity.posX;
            double dz = targetPos.zCoord - entity.posZ;
            float targetYaw = (float) Math.toDegrees(Math.atan2(dz, dx)) - 90.0f;
            float yawDiff = Math.abs(getRotationDifference(entity.rotationYaw, targetYaw));
            return yawDiff < maxAngle;
        }
    }
} 