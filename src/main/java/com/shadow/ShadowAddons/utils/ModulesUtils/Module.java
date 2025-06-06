package com.shadow.ShadowAddons.utils.ModulesUtils;


public abstract class Module {
    private boolean enabled = false;

    public abstract void onEnable();
    public abstract void onDisable();

    public boolean isEnabled() {
        return enabled;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }
}