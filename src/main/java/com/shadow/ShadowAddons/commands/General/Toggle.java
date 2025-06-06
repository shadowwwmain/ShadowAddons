package com.shadow.ShadowAddons.commands.General;

import com.shadow.ShadowAddons.ShadowAddons;
import com.shadow.ShadowAddons.utils.CommandUtils.Command;
import com.shadow.ShadowAddons.utils.CommandUtils.CommandUtils;
import com.shadow.ShadowAddons.utils.ModulesUtils.Module;
import javafx.scene.effect.Shadow;
import net.minecraft.command.CommandException;
import net.minecraft.command.ICommandSender;

public class Toggle {

    public static void loadCommands() {
        // Toggle command
        ShadowAddons.commandManager.addCommand(new Command("toggle") {
            @Override
            public void processCommand(ICommandSender sender, String[] args) throws CommandException {
                if (args.length == 0) {
                    CommandUtils.sendError("Usage: /toggle <module>");
                    return;
                }

                Module module = ShadowAddons.moduleManager.getModuleByName(args[0]);
                if (module == null) {
                    CommandUtils.sendError("Module not found: " + args[0]);
                    return;
                }

                if (module.isEnabled()) {
                    ShadowAddons.moduleManager.disableModule(module);
                    CommandUtils.sendSuccess("Disabled " + module.getClass().getSimpleName());
                } else {
                    ShadowAddons.moduleManager.enableModule(module);
                    CommandUtils.sendSuccess("Enabled " + module.getClass().getSimpleName());
                }
            }
        });
    }
}
