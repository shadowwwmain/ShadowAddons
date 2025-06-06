package com.shadow.ShadowAddons;

import com.shadow.ShadowAddons.Modules.General.ping;
import com.shadow.ShadowAddons.commands.General.Toggle;
import com.shadow.ShadowAddons.commands.Movement.Pathwalk;
import com.shadow.ShadowAddons.utils.CommandUtils.CommandManager;
import com.shadow.ShadowAddons.utils.ModulesUtils.ModuleManager;
import com.shadow.ShadowAddons.utils.pathfinding.PathfindingIntegration;
import com.shadow.ShadowAddons.utils.pathfinding.SAStarPathfinder;
import com.shadow.ShadowAddons.utils.pathfinding.old2;
import net.minecraftforge.common.MinecraftForge;
import net.minecraftforge.fml.common.Mod;
import net.minecraftforge.fml.common.event.FMLInitializationEvent;

@Mod(modid = ShadowAddons.MODID, version = ShadowAddons.VERSION, name = ShadowAddons.NAME)
public class ShadowAddons {
    public static final String MODID = "ShadowAddons";
    public static final String VERSION = "1.0";
    public static final String NAME = "Shadow Addons";
    public static ModuleManager moduleManager;
    public static CommandManager commandManager;
    private static ShadowAddons instance;
    public static PathfindingIntegration pathfinder;

    public ShadowAddons() {
        instance = this;
    }

    public static ShadowAddons getInstance() {
        return instance;
    }

    @Mod.EventHandler
    public void init(FMLInitializationEvent event) {
        MinecraftForge.EVENT_BUS.register(this);
        moduleManager = new ModuleManager();
        commandManager = new CommandManager();
        moduleManager.addModule(new ping());
        
        // Register commands
        Toggle.loadCommands();
        Pathwalk.loadCommands();
    }
} 