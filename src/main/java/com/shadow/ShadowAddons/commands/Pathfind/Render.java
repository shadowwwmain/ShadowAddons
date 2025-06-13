package com.shadow.ShadowAddons.commands.Pathfind;

import com.shadow.ShadowAddons.ShadowAddons;
import com.shadow.ShadowAddons.utils.pathfinding.SAStarPathfinder;
import net.minecraft.util.BlockPos;
import com.shadow.ShadowAddons.utils.pathfinding.PathWalker;

import java.nio.file.Path;

public class Render {
    //create a render command that renders the pathfinding overlay



    public static void loadCommands() {
        ShadowAddons.commandManager.addCommand(new com.shadow.ShadowAddons.utils.CommandUtils.Command("render") {
            @Override
            public void processCommand(net.minecraft.command.ICommandSender sender, String[] args) {
                if (args.length < 1) {
                    com.shadow.ShadowAddons.utils.CommandUtils.CommandUtils.sendError("Usage: /render <on/off>");
                    return;
                }

                SAStarPathfinder.initialize(sender.getEntityWorld());

                String option = args[0].toLowerCase();
                if (option.equals("on")) {
                    SAStarPathfinder.getInstance().setDebugEnabled(true);
                } else if (option.equals("off")) {
                    SAStarPathfinder.getInstance().setDebugEnabled(false);
                } else {
                    com.shadow.ShadowAddons.utils.CommandUtils.CommandUtils.sendError("Invalid option. Use 'on' or 'off'.");
                }
            }
        });

        ShadowAddons.commandManager.addCommand(new com.shadow.ShadowAddons.utils.CommandUtils.Command("find") {
            @Override
            public void processCommand(net.minecraft.command.ICommandSender sender, String[] args) {
                if (args.length < 3) {
                    com.shadow.ShadowAddons.utils.CommandUtils.CommandUtils.sendError("Usage: /find <x> <y> <z>");
                    return;
                }
                try {
                    int x = Integer.parseInt(args[0]);
                    int y = Integer.parseInt(args[1]);
                    int z = Integer.parseInt(args[2]);
                    SAStarPathfinder.initialize(sender.getEntityWorld());

                    BlockPos pos = new BlockPos(x, y, z);
                    SAStarPathfinder.getInstance().findPath(sender.getPosition(), pos);
                    com.shadow.ShadowAddons.utils.CommandUtils.CommandUtils.sendSuccess("Pathfinding initiated to " + x + ", " + y + ", " + z);
                } catch (NumberFormatException e) {
                    com.shadow.ShadowAddons.utils.CommandUtils.CommandUtils.sendError("Invalid coordinates. Please enter valid integers.");
                }
            }
        });

    }
}
