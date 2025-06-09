package com.shadow.ShadowAddons.utils.ModulesUtils;

import com.shadow.ShadowAddons.utils.ModulesUtils.Module;
import net.minecraftforge.common.MinecraftForge;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class ModuleManager {
    private final Map<Category, List<Module>> modulesByCategory = new HashMap<>();
    private final List<Module> modules = new ArrayList<>();

    public ModuleManager() {
        MinecraftForge.EVENT_BUS.register(this);
        // Initialize categories
        for (Category category : Category.values()) {
            modulesByCategory.put(category, new ArrayList<>());
        }
    }

    public void addModule(Module module) {
        modules.add(module);
        modulesByCategory.get(module.getCategory()).add(module);
        MinecraftForge.EVENT_BUS.register(module);
    }

    public void enableModule(Module module) {
        if (!module.isEnabled()) {
            module.onEnable();
            module.setEnabled(true);
            MinecraftForge.EVENT_BUS.register(module);
        }
    }

    public void disableModule(Module module) {
        if (module.isEnabled()) {
            module.onDisable();
            module.setEnabled(false);
            MinecraftForge.EVENT_BUS.unregister(module);
        }
    }

    public void toggleModule(Module module) {
        if (module.isEnabled()) {
            disableModule(module);
        } else {
            enableModule(module);
        }
    }

    public boolean isModuleEnabled(String name) {
        Module module = getModuleByName(name);
        return module != null && module.isEnabled();
    }

    public List<Module> getModules() {
        return modules;
    }

    public List<Module> getModulesByCategory(Category category) {
        return modulesByCategory.get(category);
    }

    public Module getModuleByName(String name) {
        for (Module module : modules) {
            if (module.getClass().getSimpleName().equalsIgnoreCase(name)) {
                return module;
            }
        }
        return null;
    }

    public enum Category {
        COMBAT("Combat"),
        RENDER("Render"),
        PLAYER("Player"),
        MOVEMENT("Movement"),
        MISC("Misc");

        private final String name;

        Category(String name) {
            this.name = name;
        }

        public String getName() {
            return name;
        }
    }
}
