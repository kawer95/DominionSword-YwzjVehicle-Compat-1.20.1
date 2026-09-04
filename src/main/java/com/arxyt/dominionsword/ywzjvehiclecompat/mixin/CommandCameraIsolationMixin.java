package com.arxyt.dominionsword.ywzjvehiclecompat.mixin;

import com.arxyt.dominionsword.client.ClientSpirit;
import com.arxyt.dominionsword.ywzjvehiclecompat.client.CommandCameraGuard;
import net.minecraft.client.Camera;
import net.minecraft.world.entity.Entity;
import net.minecraft.world.level.BlockGetter;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

/** Captures vanilla's detached-camera transform before YWZJ's setup-tail override runs. */
@Mixin(value = Camera.class, priority = 500)
public abstract class CommandCameraIsolationMixin {
    @Inject(method = "setup", at = @At(value = "INVOKE",
            target = "Lnet/minecraft/client/Camera;setPosition(DDD)V", shift = At.Shift.AFTER))
    private void dominionsword$captureCommandCamera(BlockGetter level, Entity entity, boolean detached,
                                                     boolean reverse, float partialTick, CallbackInfo ci) {
        if (ClientSpirit.detachedCameraTransition()) CommandCameraGuard.capture((Camera) (Object) this);
    }
}
