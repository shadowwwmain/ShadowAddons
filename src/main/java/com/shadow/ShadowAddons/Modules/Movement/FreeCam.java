package com.shadow.ShadowAddons.Modules.Movement;

import com.shadow.ShadowAddons.utils.Keybinds.KeyBindUtils;
import com.shadow.ShadowAddons.utils.ModulesUtils.Module;
import com.shadow.ShadowAddons.utils.ModulesUtils.ModuleManager;
import net.minecraft.client.Minecraft;
import net.minecraft.client.entity.EntityPlayerSP;
import net.minecraft.client.settings.KeyBinding;
import net.minecraft.network.Packet;
import net.minecraft.network.play.client.*;
import net.minecraftforge.common.MinecraftForge;
import net.minecraftforge.fml.common.eventhandler.SubscribeEvent;
import net.minecraftforge.fml.common.gameevent.InputEvent;
import net.minecraftforge.fml.common.gameevent.TickEvent;
import net.minecraftforge.fml.common.network.FMLNetworkEvent;
import com.shadow.ShadowAddons.utils.ModulesUtils.ModuleManager;
import org.lwjgl.input.Keyboard;

public class FreeCam extends Module {

    private boolean noClip = true;
    private boolean flyMode = true;
    private float speed = 1f;
    Minecraft mc = Minecraft.getMinecraft();
    private double savedX, savedY, savedZ;
    private float savedYaw, savedPitch;
    private boolean savedSprinting;


    @Override
    public void onEnable() {
        EntityPlayerSP player = mc.thePlayer;
        if (player == null) return;

        savedX = player.posX;
        savedY = player.posY;
        savedZ = player.posZ;
        savedYaw = player.rotationYaw;
        savedPitch = player.rotationPitch;
        savedSprinting = player.isSprinting();

        player.noClip = noClip;

        MinecraftForge.EVENT_BUS.register(this);
    }

    @Override
    public void onDisable() {
        EntityPlayerSP player = mc.thePlayer;
        if (player == null) return;

        player.setPositionAndRotation(savedX, savedY, savedZ, savedYaw, savedPitch);
        player.motionX = 0;
        player.motionY = 0;
        player.motionZ = 0;
        player.noClip = false;
        player.setSprinting(savedSprinting);

        MinecraftForge.EVENT_BUS.unregister(this);
    }

    @SubscribeEvent
    public void onClientTick(TickEvent.ClientTickEvent event) {
        if (!isToggled() || mc.thePlayer == null || mc.theWorld == null) return;

        EntityPlayerSP player = mc.thePlayer;
        player.noClip = noClip;
        player.motionX = 0;
        player.motionY = 0;
        player.motionZ = 0;

        float flySpeed = speed / 2.0f;

        if (mc.gameSettings.keyBindJump.isKeyDown()) {
            player.motionY += flySpeed;
        }

        if (mc.gameSettings.keyBindSneak.isKeyDown()) {
            player.motionY -= flySpeed;
        }

        if (flyMode) {
            if (mc.gameSettings.keyBindForward.isKeyDown()) {
                player.motionX -= Math.sin(Math.toRadians(player.rotationYaw)) * flySpeed;
                player.motionZ += Math.cos(Math.toRadians(player.rotationYaw)) * flySpeed;
            }

            if (mc.gameSettings.keyBindBack.isKeyDown()) {
                player.motionX += Math.sin(Math.toRadians(player.rotationYaw)) * flySpeed;
                player.motionZ -= Math.cos(Math.toRadians(player.rotationYaw)) * flySpeed;
            }

            if (mc.gameSettings.keyBindLeft.isKeyDown()) {
                player.motionX += Math.sin(Math.toRadians(player.rotationYaw - 90)) * flySpeed;
                player.motionZ -= Math.cos(Math.toRadians(player.rotationYaw - 90)) * flySpeed;
            }

            if (mc.gameSettings.keyBindRight.isKeyDown()) {
                player.motionX += Math.sin(Math.toRadians(player.rotationYaw + 90)) * flySpeed;
                player.motionZ -= Math.cos(Math.toRadians(player.rotationYaw + 90)) * flySpeed;
            }
        }
    }

    @SubscribeEvent
    public void onPacketSend(FMLNetworkEvent.ClientCustomPacketEvent event) {
        // Optional: cancel specific packets
    }

    //create isToggled method to check if the module is enabled
    private boolean isToggled() {
        if(Module.isEnabled()) {
            return true;
        } else {
            return false;
        }
    }

        @SubscribeEvent
        public void onInput(InputEvent event) {
            if (isToggled() && mc.thePlayer != null) {
                mc.thePlayer.movementInput.moveForward = 0;
                mc.thePlayer.movementInput.moveStrafe = 0;
                mc.thePlayer.movementInput.jump = false;
            }
        }



    // Optional setters to integrate with config GUI
    public void setSpeed(float speed) {
        this.speed = speed;
    }

    public void setNoClip(boolean noClip) {
        this.noClip = noClip;
    }

    public void setFlyMode(boolean flyMode) {
        this.flyMode = flyMode;
    }

    @Override
    public ModuleManager.Category getCategory() {
        return ModuleManager.Category.MOVEMENT;
    }
}
