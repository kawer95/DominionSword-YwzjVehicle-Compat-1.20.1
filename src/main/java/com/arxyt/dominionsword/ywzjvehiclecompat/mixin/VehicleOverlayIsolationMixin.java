package com.arxyt.dominionsword.ywzjvehiclecompat.mixin;

import com.arxyt.dominionsword.client.ClientSpirit;
import com.arxyt.dominionsword.ywzjvehiclecompat.CommandCameraHudPolicy;
import net.minecraftforge.client.event.RenderGuiOverlayEvent;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;
import org.ywzj.vehicle.client.handler.OverlayHandler;

/** Keeps Dominion's HUD and cursor render layers alive while the player remains seated in YWZJ. */
@Mixin(value = OverlayHandler.class, remap = false)
public abstract class VehicleOverlayIsolationMixin {
    @Inject(method = "onRenderOverlay", at = @At("HEAD"), cancellable = true)
    private static void dominionsword$keepCommandCameraHud(RenderGuiOverlayEvent.Pre event, CallbackInfo ci) {
        if (CommandCameraHudPolicy.bypassMountedHudSuppression(ClientSpirit.detachedCameraTransition())) {
            ci.cancel();
        }
    }
}
