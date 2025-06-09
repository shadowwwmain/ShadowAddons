package com.shadow.ShadowAddons.utils.MiningUtil;

import com.shadow.ShadowAddons.utils.RotationsUtils.RotationHandler;
import com.sun.glass.ui.Robot;
import net.minecraft.block.material.Material;
import net.minecraft.block.state.IBlockState;
import net.minecraft.client.Minecraft;
import net.minecraft.item.ItemStack;
import net.minecraft.nbt.NBTTagList;
import net.minecraft.util.BlockPos;
import net.minecraft.util.ChatComponentText;
import net.minecraft.util.EnumFacing;
import net.minecraft.util.MovingObjectPosition;
import org.lwjgl.input.Mouse;

import java.awt.event.InputEvent;
import java.lang.reflect.Method;


public class MiningUtils {

    public static void mineBlockAt(int x, int y, int z) {
        Minecraft mc = Minecraft.getMinecraft();
        BlockPos pos = new BlockPos(x, y, z);
        RotationHandler.smoothRotate(mc.thePlayer, pos);
        MiningUtils.simulateLeftClick(pos);
    }

    public static void simulateLeftClick(BlockPos pos) {
        Minecraft mc = Minecraft.getMinecraft();

        if (mc.theWorld == null || mc.thePlayer == null) return;

        // Check if block exists
        IBlockState blockState = mc.theWorld.getBlockState(pos);
        if (blockState.getBlock().getMaterial() == Material.air) return;

        // Start breaking the block
        mc.playerController.onPlayerDamageBlock(pos, EnumFacing.UP);

        // Continue breaking (you might want to call this multiple times)
        mc.playerController.onPlayerDamageBlock(pos, EnumFacing.UP);

        // Finish breaking the block
        mc.playerController.clickBlock(pos, EnumFacing.UP);
    }
    public static BlockPos getBlockPlayerIsLookingAt() {
        Minecraft mc = Minecraft.getMinecraft();
        MovingObjectPosition mop = mc.objectMouseOver;
        if (mop != null && mop.typeOfHit == MovingObjectPosition.MovingObjectType.BLOCK) {
            return mop.getBlockPos();
        }
        return null; // Not looking at a block
    }


    public static boolean isLookingAtBedrock() {
        BlockPos pos = getBlockPlayerIsLookingAt();
        if (pos != null) {
            return Minecraft.getMinecraft().theWorld.getBlockState(pos).getBlock().getMaterial().isSolid() &&
                    Minecraft.getMinecraft().theWorld.getBlockState(pos).getBlock().getUnlocalizedName().contains("bedrock");
        }
        return false; // Not looking at a block or the block is not bedrock
    }

    public static boolean isLookingAtObsidian() {
        BlockPos pos = getBlockPlayerIsLookingAt();
        if (pos != null) {
            return Minecraft.getMinecraft().theWorld.getBlockState(pos).getBlock().getMaterial().isSolid() &&
                    Minecraft.getMinecraft().theWorld.getBlockState(pos).getBlock().getUnlocalizedName().contains("obsidian");
        }
        return false; // Not looking at a block or the block is not obsidian
    }

    public static void swapAndUseAbility() {
        Minecraft mc = Minecraft.getMinecraft();
        int originalSlot = -1;
        int swapSlot = -1;

        // Find original and swap shards in hotbar
        for (int i = 0; i < 9; i++) {
            ItemStack stack = mc.thePlayer.inventory.getStackInSlot(i);
            if (stack == null) continue;
            String name = stack.getDisplayName();
            if ((name.contains("655") || name.contains("Divan's")) && originalSlot == -1) {
                originalSlot = i;
            } else if (isPrismarineShard(stack) && hasBlueCheeseGoblinLore(stack) && swapSlot == -1) {
                swapSlot = i;
            }
        }

        if (originalSlot == -1 || swapSlot == -1) {
            mc.thePlayer.addChatMessage(new ChatComponentText("Could not find required shards in hotbar."));
            return;
        }

        // Swap to the blue cheese goblin shard
        mc.thePlayer.inventory.currentItem = swapSlot;
        mc.playerController.updateController();
        ItemStack heldItem = mc.thePlayer.inventory.getCurrentItem();
        int facing = mc.objectMouseOver.sideHit.getIndex();
        BlockPos pos = mc.objectMouseOver.getBlockPos();

        mc.thePlayer.sendQueue.addToSendQueue(
                new net.minecraft.network.play.client.C08PacketPlayerBlockPlacement(
                        pos, facing, heldItem, 0.5f, 0.5f, 0.5f
                ));

        // Swap back to the original shard
        mc.thePlayer.inventory.currentItem = originalSlot;
        mc.playerController.updateController();
    }

    private static boolean isPrismarineShard(ItemStack stack) {
        return stack.getUnlocalizedName().toLowerCase().contains("prismarine_shard");
    }

    private static boolean hasBlueCheeseGoblinLore(ItemStack stack) {
        if (stack.hasTagCompound() && stack.getTagCompound().hasKey("display", 10)) {
            NBTTagList lore = stack.getTagCompound().getCompoundTag("display").getTagList("Lore", 8);
            for (int i = 0; i < lore.tagCount(); i++) {
                String line = lore.getStringTagAt(i).toLowerCase();
                if (line.contains("blue cheese goblin")) {
                    return true;
                }
            }
        }
        return false;
    }




}
