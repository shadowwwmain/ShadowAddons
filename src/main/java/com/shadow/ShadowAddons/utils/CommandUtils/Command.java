package com.shadow.ShadowAddons.utils.CommandUtils;

import net.minecraft.command.CommandBase;
import net.minecraft.command.CommandException;
import net.minecraft.command.ICommandSender;

public abstract class Command extends CommandBase {
    private String name;
    private String[] aliases;

    public Command(String name, String... aliases) {
        this.name = name;
        this.aliases = aliases;
    }

    @Override
    public String getCommandName() {
        return name;
    }

    @Override
    public String getCommandUsage(ICommandSender sender) {
        return "/" + name;
    }

    @Override
    public int getRequiredPermissionLevel() {
        return 0;
    }

    @Override
    public abstract void processCommand(ICommandSender sender, String[] args) throws CommandException;
}
