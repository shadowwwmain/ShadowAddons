package com.shadow.ShadowAddons.commands.Pathfind;

import com.shadow.ShadowAddons.ShadowAddons;
import com.shadow.ShadowAddons.utils.CommandUtils.CommandUtils;
import com.shadow.ShadowAddons.utils.pathfinding.PathWalker;
import net.minecraft.command.ICommandSender;
import com.shadow.ShadowAddons.utils.CommandUtils.Command;
import net.minecraft.util.BlockPos;

public class Test {
    // Load the test command
    public static void loadCommands() {
        ShadowAddons.commandManager.addCommand(new Command("test") {
            @Override
            public void processCommand(ICommandSender sender, String[] args) {
                if (args.length < 2) {
                    CommandUtils.sendError("Usage: /test <x> <y> <z>");
                    return;
                }
                PathWalker walker = PathWalker.getInstance();

                try {
                    int x = Integer.parseInt(args[0]);
                    int y = Integer.parseInt(args[1]);
                    int z = Integer.parseInt(args[2]);

                    BlockPos targetPos = new BlockPos(x, y, z);

                    if (walker.isWalking()) {
                        CommandUtils.sendError("Already walking a path. Please stop the current path before starting a new one.");
                        return;
                    }
                    walker.intliaizePathfinder();

                    walker.startWalking(targetPos);


                } catch (NumberFormatException e) {
                    CommandUtils.sendError("Invalid coordinates. Please enter valid integers.");
                }
            }
        });
    }



}