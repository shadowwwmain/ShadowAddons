package com.shadow.ShadowAddons.utils.ModulesUtils;

import net.minecraftforge.common.MinecraftForge;

import java.util.ArrayList;
import java.util.List;

public class ModuleManager {
    private List<Module> modules = new ArrayList<>();

    public ModuleManager() {
        MinecraftForge.EVENT_BUS.register(this);
    }

    public void addModule(Module module) {
        modules.add(module);
        MinecraftForge.EVENT_BUS.register(module);
    }

    public void enableModule(Module module) {
        module.onEnable();
        module.setEnabled(true);
    }

    public void disableModule(Module module) {
        module.onDisable();
        module.setEnabled(false);
    }

    public List<Module> getModules() {
        return modules;
    }

    public Module getModuleByName(String name) {
        for (Module module : modules) {
            if (module.getClass().getSimpleName().equalsIgnoreCase(name)) {
                return module;
            }
        }
        return null;
    }


}