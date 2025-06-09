package com.shadow.ShadowAddons.commands.General;

import com.shadow.ShadowAddons.ShadowAddons;
import com.shadow.ShadowAddons.utils.CommandUtils.Command;
import com.shadow.ShadowAddons.utils.CommandUtils.CommandUtils;
import com.shadow.ShadowAddons.utils.Keybinds.KeyBindUtils;
import com.shadow.ShadowAddons.utils.MiningUtil.MiningUtils;
import com.shadow.ShadowAddons.utils.RotationsUtils.RotationHandler;
import net.minecraft.command.CommandException;
import net.minecraft.command.ICommandSender;
import net.minecraft.util.BlockPos;

public class Rotate {
    public static void loadCommands() {
        //create rotate command
        ShadowAddons.commandManager.addCommand(new Command("rotate") {
            @Override
            public void processCommand(ICommandSender sender, String[] args) throws CommandException {
                RotationHandler.rotateTo2BlocksAhead();
//close the command
                CommandUtils.sendSuccess("Rotated to 2 blocks ahead");
                MiningUtils.mineBlockAt(sender.getPosition().getX(),
                        sender.getPosition().getY(),
                        sender.getPosition().getZ());
                BlockPos pos2 = new BlockPos(MiningUtils.getBlockPlayerIsLookingAt());
                CommandUtils.sendError(pos2.toString());
                KeyBindUtils.getKeyBind("chat").setKeyCode(1);
            }
        });
    }
}