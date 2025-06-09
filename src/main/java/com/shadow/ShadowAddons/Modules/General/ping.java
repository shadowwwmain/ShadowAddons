package com.shadow.ShadowAddons.Modules.General;

import com.shadow.ShadowAddons.utils.ModulesUtils.Module;
import com.shadow.ShadowAddons.utils.ModulesUtils.ModuleManager;
import net.minecraft.util.EnumChatFormatting;
import net.minecraftforge.fml.common.gameevent.TickEvent;
import net.minecraftforge.fml.common.eventhandler.SubscribeEvent;
import net.minecraftforge.common.MinecraftForge;
import com.shadow.ShadowAddons.utils.MessageUtils.MessageUtil;

import java.awt.*;

import static com.shadow.ShadowAddons.utils.MessageUtils.MessageUtil.sendColoredMessage;

public class ping extends Module {

    @Override
    public void onEnable() {

    }

    @Override
    public void onDisable() {

    }

    @Override
    public ModuleManager.Category getCategory() {
        return ModuleManager.Category.MISC;
    }


}