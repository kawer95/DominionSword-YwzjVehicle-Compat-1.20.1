package com.arxyt.dominionsword.ywzjvehiclecompat.mixin;

import com.arxyt.dominionsword.client.ClientSpirit;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfoReturnable;
import org.ywzj.vehicle.vehicle.LocalVehiclePlayer;

/** Lets Dominion's command-camera input run instead of YWZJ's mounted-view mouse lock. */
@Mixin(value = LocalVehiclePlayer.class, remap = false)
public abstract class VehicleMouseIsolationMixin {
    @Inject(method = "handlePlayerTurn", at = @At("HEAD"), cancellable = true)
    private void dominionsword$releaseCommandCameraMouse(double yaw, double pitch,
                                                         CallbackInfoReturnable<Boolean> cir) {
        if (ClientSpirit.detachedCameraTransition()) cir.setReturnValue(false);
    }
}
