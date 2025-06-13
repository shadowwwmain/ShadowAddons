package com.shadow.ShadowAddons.utils.FailsafesUtil;

import com.shadow.ShadowAddons.utils.MessageUtils.MessageUtil;
import net.minecraft.util.EnumChatFormatting;

public abstract class Failsafe {
    protected boolean toggled = false;
    protected boolean triggered = false;
    protected long responseStartTime = 0;
    protected long waitTime = 0;

    public void setToggled(boolean toggle) {
        this.toggled = toggle;
        MessageUtil.sendColoredMessage("[Failsafe] " + this.getClass().getSimpleName() + " toggled: " + toggle, EnumChatFormatting.AQUA);
        reset();

        if (toggle) registerTriggers();
        else unregisterTriggers();
    }

    public boolean isToggled() {
        return toggled;
    }

    public void reset() {
        triggered = false;
    }

    protected abstract void registerTriggers();
    protected abstract void unregisterTriggers();
}
