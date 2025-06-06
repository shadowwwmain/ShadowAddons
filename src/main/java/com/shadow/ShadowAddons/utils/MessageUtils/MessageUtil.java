package com.shadow.ShadowAddons.utils.MessageUtils;

import net.minecraft.util.EnumChatFormatting;

import static com.shadow.ShadowAddons.utils.CommandUtils.CommandUtils.sendMessage;

public class MessageUtil {

   public static void sendColoredMessage(String message, EnumChatFormatting color) {
        sendMessage(color + message);
    }


}
