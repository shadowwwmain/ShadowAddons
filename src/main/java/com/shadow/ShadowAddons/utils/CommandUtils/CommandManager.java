package com.shadow.ShadowAddons.utils.CommandUtils;

import net.minecraftforge.client.ClientCommandHandler;
import java.util.ArrayList;
import java.util.List;

public class CommandManager {
    private List<Command> commands = new ArrayList<Command>();

    public void addCommand(Command command) {
        commands.add(command);
        ClientCommandHandler.instance.registerCommand(command);
    }

    public List<Command> getCommands() {
        return commands;
    }
}
