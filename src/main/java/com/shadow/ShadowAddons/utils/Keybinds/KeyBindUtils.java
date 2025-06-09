package com.shadow.ShadowAddons.utils.Keybinds;

import net.minecraft.client.Minecraft;
import net.minecraft.client.settings.KeyBinding;

public class KeyBindUtils {
    public static KeyBinding getKeyBind(String keyName) {
        Minecraft mc = Minecraft.getMinecraft();
        switch (keyName.toLowerCase()) {
            case "forward":
                return mc.gameSettings.keyBindForward;
            case "back":
                return mc.gameSettings.keyBindBack;
            case "left":
                return mc.gameSettings.keyBindLeft;
            case "right":
                return mc.gameSettings.keyBindRight;
            case "sneak":
                return mc.gameSettings.keyBindSneak;
            case "jump":
                return mc.gameSettings.keyBindJump;
            case "sprint":
                return mc.gameSettings.keyBindSprint;
            case "use_item":
                return mc.gameSettings.keyBindUseItem;
            case "attack":
                return mc.gameSettings.keyBindAttack;
            case "drop":
                return mc.gameSettings.keyBindDrop;
            case "inventory":
                return mc.gameSettings.keyBindInventory;
            case "chat":
                return mc.gameSettings.keyBindChat;
            case "toggle_perspective":
                return mc.gameSettings.keyBindTogglePerspective;
            default:
                return null; // or throw an exception
        }
    }
}
