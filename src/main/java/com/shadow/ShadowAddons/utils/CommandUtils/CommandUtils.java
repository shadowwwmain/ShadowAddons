package com.shadow.ShadowAddons.utils.CommandUtils;

import net.minecraft.client.Minecraft;
import net.minecraft.util.ChatComponentText;
import net.minecraft.util.EnumChatFormatting;

public class CommandUtils {

    public static void sendMessage(String message) {
        Minecraft.getMinecraft().thePlayer.addChatMessage(new ChatComponentText(message));
    }

    public static void sendMessage(String message, EnumChatFormatting color) {
        sendMessage(color + message);
    }

    public static void sendSuccess(String message) {
        sendMessage(EnumChatFormatting.GREEN + message);
    }

    public static void sendError(String message) {
        sendMessage(EnumChatFormatting.RED + message);
    }

    public static void sendInfo(String message) {
        sendMessage(EnumChatFormatting.YELLOW + message);
    }
}
