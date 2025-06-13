//package com.shadow.ShadowAddons.utils.FailsafesUtil;
//
//import com.shadow.ShadowAddons.utils.MessageUtils.MessageUtil;
//import jdk.nashorn.internal.ir.EmptyNode;
//import net.minecraft.client.settings.KeyBinding;
//import net.minecraft.util.EnumChatFormatting;
//import net.minecraftforge.fml.client.registry.ClientRegistry;
//
//import java.util.HashMap;
//import java.util.Map;
//import java.util.Set;
//
//public class FailsafesManager {
//    private final Map<String, Failsafe> failsafes = new HashMap<>();
//    private boolean failsafesEnabled = true;
//    private boolean responsesEnabled = true;
//    private int responseWaitTime = 2000;
//    private boolean overlayEnabled = true;
//    private String sensitivity = "Normal";
//
//    private long responseStartTime = 0;
//
//    public FailsafesManager() {
//        // Example of registering failsafes
//        failsafes.put("Rotation", new RotationFailsafe());
//        failsafes.put("Teleport", new TeleportFailsafe());
//        failsafes.put("Velocity", new VelocityFailsafe());
//        failsafes.put("Player", new PlayerFailsafe());
//        failsafes.put("Block", new BlockFailsafe());
//        failsafes.put("Item", new ItemFailsafe());
//        failsafes.put("Server", new ServerFailsafe());
//        failsafes.put("Smart", new SmartFailsafe());
//
//        registerOverlay();
//
//        KeyBinding cancelKey = KeyBindRegistry.get("Cancel Response");
//        ClientRegistry.registerKeyBinding(cancelKey);
//    }
//
//    public void registerFailsafes(Runnable triggerAction, Runnable postAction, Set<String> options) {
//        unregisterFailsafes();
//        for (String name : options) {
//            Failsafe f = failsafes.get(name);
//            if (f != null) {
//                f.setToggled(true);
//            }
//        }
//    }
//
//    public void unregisterFailsafes() {
//        for (Failsafe failsafe : failsafes.values()) {
//            failsafe.setToggled(false);
//        }
//    }
//
//    public void trigger(String failsafeName, String message) {
//        if (!isFailsafeAllowed()) return;
//
//        unregisterFailsafes();
//        failsafes.get("Smart").reset();
//
//        WebhookManager.sendMessage(failsafeName);
//
//        MessageUtil.sendColoredMessage("AutoVegetable", "§c" + failsafeName + " " + message, EnumChatFormatting.RED);
//
//        if (!responsesEnabled) return;
//
//        responseStartTime = System.currentTimeMillis();
//        if (overlayEnabled) {
//            OverlayManager.enableOverlay("FAILSAFES");
//            OverlayManager.updateOverlayBar("FAILSAFES", "RESPONSE_TIMER", 0f, "Responding in " + (responseWaitTime / 1000f) + "s...");
//        }
//
//        new Thread(() -> {
//            try {
//                Thread.sleep(responseWaitTime);
//                ResponseBot.findOptimalResponse(() -> {
//                    OverlayManager.disableOverlay("FAILSAFES");
//                });
//            } catch (InterruptedException ignored) {}
//        }).start();
//    }
//
//    private boolean isFailsafeAllowed() {
//        if (!failsafesEnabled) return false;
//        if (ResponseBot.isPlayingBack()) return false;
//        if (System.currentTimeMillis() - responseStartTime < responseWaitTime) return false;
//        return !AutoReconnect.isReconnecting();
//    }
//
//    private void registerOverlay() {
//        OverlayManager.addOverlay("Failsafes", "FAILSAFES");
//        OverlayManager.addOverlayText("FAILSAFES", "ESCAPE_ALERT", "Press ESC to stop movement!");
//        OverlayManager.addOverlayBar("FAILSAFES", "RESPONSE_TIMER", 0f, "Responding soon...");
//    }
//
//    public void updateFromSettings() {
//        this.failsafesEnabled = ModuleManager.getSetting("Failsafes", "Failsafes Enabled");
//        this.responsesEnabled = ModuleManager.getSetting("Failsafes", "Responses Enabled");
//        this.responseWaitTime = ModuleManager.getSettingInt("Failsafes", "Response Delay");
//        this.overlayEnabled = ModuleManager.getSetting("Failsafes", "Responses Enabled");
//        this.sensitivity = ModuleManager.getSetting("Failsafes", "Failsafe Sensitivity");
//    }
//}
