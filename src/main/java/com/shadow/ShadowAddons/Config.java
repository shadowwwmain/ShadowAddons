package com.shadow.ShadowAddons;

import net.minecraftforge.common.config.Configuration;

import java.io.File;

public class Config {
    private static Configuration config;
    
    public static void init(File configFile) {
        config = new Configuration(configFile);
        loadConfig();
    }
    
    public static void loadConfig() {
        config.load();
        if (config.hasChanged()) {
            config.save();
        }
    }
    
    public static void saveConfig() {
        config.save();
    }
} 