package com.shadow.ShadowAddons.utils.ModulesUtils;


public abstract class Module {
    private static boolean enabled = false;

    public abstract void onEnable();
    public abstract void onDisable();

    // create getCategory method

    //create SetCategory method to whatever is in the enum Category
    public abstract ModuleManager.Category getCategory();

    public static boolean isEnabled() {
        return enabled;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }
}