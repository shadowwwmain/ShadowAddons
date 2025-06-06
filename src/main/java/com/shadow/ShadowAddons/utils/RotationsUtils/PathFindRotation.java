package com.shadow.ShadowAddons.utils.RotationsUtils;

import net.minecraft.entity.EntityLivingBase;
import net.minecraft.util.MathHelper;
import net.minecraft.util.Vec3;

public class PathFindRotation {
    public static class RotationUtils {
        private static final float ROTATION_SPEED = 10.0f;
        private static float lastYaw = 0;

        public static void rotateToPosition(EntityLivingBase entity, Vec3 targetPos) {
            double dx = targetPos.xCoord - entity.posX;
            double dz = targetPos.zCoord - entity.posZ;
            float targetYaw = (float) Math.toDegrees(Math.atan2(dz, dx)) - 90.0f;

            // Smooth rotation with interpolation
            float yawDiff = MathHelper.wrapAngleTo180_float(targetYaw - entity.rotationYaw);
            if (Math.abs(yawDiff) > 1.0f) {
                entity.rotationYaw += yawDiff * 0.1f;
            } else {
                entity.rotationYaw = targetYaw;
            }
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
