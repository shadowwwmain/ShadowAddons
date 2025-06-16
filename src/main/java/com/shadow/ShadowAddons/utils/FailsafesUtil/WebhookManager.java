package com.shadow.ShadowAddons.utils.FailsafesUtil;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import net.minecraft.client.Minecraft;
import net.minecraft.util.ScreenShotHelper;
import net.minecraftforge.fml.common.eventhandler.SubscribeEvent;
import net.minecraftforge.client.event.ClientChatReceivedEvent;

import java.awt.*;
import java.io.*;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.file.Files;
import java.util.*;
import java.util.List;

public class WebhookManager {
    private static final Minecraft mc = Minecraft.getMinecraft();
    private static final Gson gson = new Gson();

    private static String webhookUrl = "";
    private static boolean isEnabled = false;
    private static String userId = "";

    private static final Map<String, Integer> COLOR_MAP = new HashMap<String, Integer>() {{
        put("red", 0xFF0000);
        put("green", 0x00FF00);
        put("blue", 0x0000FF);
        put("yellow", 0xFFFF00);
        put("purple", 0xC71585);
        put("pink", 0xFF69B4);
        put("orange", 0xFFA500);
        put("white", 0xFFFFFF);
        put("gray", 0x808080);
        put("brown", 0xA52A2A);
        put("cyan", 0x00FFFF);
        put("lime", 0x00FF00);
        put("magenta", 0xFF00FF);
        put("teal", 0x008080);
        put("lavender", 0xE6E6FA);
        put("maroon", 0x800000);
    }};

    public WebhookManager() {
        loadConfig();
    }

    private void loadConfig() {
        try {
            File config = new File("config/webhook.json");
            if (config.exists()) {
                Map<String, String> data = gson.fromJson(new FileReader(config), new TypeToken<Map<String, String>>(){}.getType());
                webhookUrl = data.getOrDefault("url", "");
                userId = data.getOrDefault("userId", "");
                isEnabled = !webhookUrl.isEmpty();
                System.out.println("[Webhook] Loaded webhook from config.");
            }
        } catch (Exception e) {
            System.err.println("[Webhook] Failed to load webhook config.");
        }
    }

    private void saveConfig() {
        try {
            File config = new File("config/webhook.json");
            Map<String, String> data = new HashMap<>();
            data.put("url", webhookUrl);
            data.put("userId", userId);
            gson.toJson(data, new FileWriter(config));
        } catch (IOException e) {
            System.err.println("[Webhook] Failed to save webhook config.");
        }
    }

    public void setWebhook(String url) {
        if (!url.startsWith("https://discord.com/api/webhooks/")) {
            System.out.println("[Webhook] Invalid URL.");
            return;
        }
        this.webhookUrl = url;
        this.isEnabled = true;
        saveConfig();
        System.out.println("[Webhook] Webhook set.");
    }

    public void setUserId(String id) {
        this.userId = id;
        saveConfig();
        System.out.println("[Webhook] User ID set.");
    }

    public static void sendMessage(String message) {
        if (!isEnabled) return;
        try {
            Map<String, Object> body = new HashMap<>();
            body.put("username", "rdbt v4");
            body.put("content", message);
            body.put("avatar_url", "https://minotar.net/cube/" + mc.getSession().getProfile().getId().toString().replace("-", "") + "/100.png");

            sendJson(webhookUrl, gson.toJson(body));
        } catch (Exception e) {
            System.err.println("[Webhook] Failed to send message: " + e.getMessage());
        }
    }

    public static void sendMessageEmbed(String message, String embedContent, String colorName) {
        if (!isEnabled) return;

        int color = COLOR_MAP.getOrDefault(colorName.toLowerCase(), 0);

        Map<String, Object> embed = new HashMap<>();
        embed.put("title", "Rdbt V4");
        embed.put("description", embedContent);
        embed.put("color", color);

        Map<String, Object> payload = new HashMap<>();
        payload.put("username", mc.getSession().getUsername());
        payload.put("content", message);
        payload.put("avatar_url", "https://minotar.net/cube/" + mc.getSession().getProfile().getId().toString().replace("-", "") + "/100.png");
        payload.put("embeds", Collections.singletonList(embed));

        sendJson(webhookUrl, gson.toJson(payload));
    }

    public static void sendMessageWithPing(String msg) {
        if (userId == null || userId.isEmpty()) {
            System.out.println("[Webhook] No user ID set.");
            return;
        }
        sendMessage("<@" + userId + "> " + msg);
    }

    public static void sendMessageWithPingEmbed(String msg, String embed, String color) {
        if (userId == null || userId.isEmpty()) {
            System.out.println("[Webhook] No user ID set.");
            return;
        }
        sendMessageEmbed("<@" + userId + "> " + msg, embed, color);
    }

    public static void sendScreenshot(String macroName) {
        try {
            File dir = new File(mc.mcDataDir, "screenshots");
            String fileName = UUID.randomUUID().toString().replace("-", "") + ".png";
            File screenshotFile = new File(dir, fileName);

            ScreenShotHelper.saveScreenshot(mc.mcDataDir, mc.displayWidth, mc.displayHeight, mc.getFramebuffer());

            // Wait for file to be written (optional)
            Thread.sleep(1000);

            if (!screenshotFile.exists()) {
                System.err.println("[Webhook] Screenshot file not found.");
                return;
            }

            byte[] imageData = Files.readAllBytes(screenshotFile.toPath());

            // Discord does not support raw Base64 screenshots via HTTP, you'd need Apache HttpClient or similar for multipart.
            // This part needs external libraries or custom multipart code.
            System.out.println("[Webhook] Screenshot logic present but multipart POST is not yet implemented.");
        } catch (Exception e) {
            System.err.println("[Webhook] Failed to take screenshot: " + e.getMessage());
        }
    }

    private static void sendJson(String url, String jsonPayload) {
        try {
            URL webhook = new URL(url);
            HttpURLConnection conn = (HttpURLConnection) webhook.openConnection();
            conn.setRequestMethod("POST");
            conn.setRequestProperty("Content-Type", "application/json");
            conn.setRequestProperty("User-Agent", "Mozilla/5.0");
            conn.setDoOutput(true);

            try (OutputStream os = conn.getOutputStream()) {
                os.write(jsonPayload.getBytes());
                os.flush();
            }

            int responseCode = conn.getResponseCode();
            if (responseCode != 204 && responseCode != 200) {
                System.err.println("[Webhook] Discord webhook response code: " + responseCode);
            }
        } catch (IOException e) {
            System.err.println("[Webhook] Error sending webhook: " + e.getMessage());
        }
    }
}
