package com.arxyt.dominionsword.ywzjvehiclecompat.mixin;

import com.arxyt.dominionsword.client.ClientSpirit;
import net.minecraft.client.Camera;
import net.minecraft.world.entity.Entity;
import net.minecraft.world.level.BlockGetter;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Unique;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;
import org.ywzj.vehicle.entity.vehicle.AbstractVehicle;
import org.ywzj.vehicle.vehicle.LocalVehiclePlayer;

/** Prevents YWZJ's Camera tail injection from replacing Dominion's detached camera transform. */
@Mixin(value = Camera.class, priority = 500)
public abstract class CommandCameraIsolationMixin {
    @Unique private AbstractVehicle dominionsword$vehicle;
    @Unique private AbstractVehicle.Seat dominionsword$seat;
    @Unique private boolean dominionsword$suspended;

    @Inject(method = "setup", at = @At("HEAD"))
    private void dominionsword$suspendVehicleCamera(BlockGetter level, Entity entity, boolean detached,
                                                     boolean reverse, float partialTick, CallbackInfo ci) {
        if (!ClientSpirit.active()) return;
        LocalVehiclePlayer state = LocalVehiclePlayer.instance;
        if (state == null || state.vehicle == null) return;
        dominionsword$vehicle = state.vehicle;
        dominionsword$seat = state.seat;
        dominionsword$suspended = true;
        state.vehicle = null;
        state.seat = null;
    }

    @Inject(method = "setup", at = @At("RETURN"))
    private void dominionsword$restoreVehicleCamera(BlockGetter level, Entity entity, boolean detached,
                                                     boolean reverse, float partialTick, CallbackInfo ci) {
        if (!dominionsword$suspended) return;
        LocalVehiclePlayer state = LocalVehiclePlayer.instance;
        state.vehicle = dominionsword$vehicle;
        state.seat = dominionsword$seat;
        dominionsword$vehicle = null;
        dominionsword$seat = null;
        dominionsword$suspended = false;
    }
}
