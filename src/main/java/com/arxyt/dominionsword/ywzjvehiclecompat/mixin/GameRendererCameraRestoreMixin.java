package com.arxyt.dominionsword.ywzjvehiclecompat.mixin;

import com.arxyt.dominionsword.ywzjvehiclecompat.client.CommandCameraGuard;
import com.mojang.blaze3d.vertex.PoseStack;
import net.minecraft.client.renderer.GameRenderer;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

/** Restores Dominion's transform only after every Camera.setup tail injector has completed. */
@Mixin(value = GameRenderer.class, priority = 500)
public abstract class GameRendererCameraRestoreMixin {
    @Inject(method = "renderLevel", at = @At(value = "INVOKE",
            target = "Lnet/minecraft/client/Camera;setup(Lnet/minecraft/world/level/BlockGetter;Lnet/minecraft/world/entity/Entity;ZZF)V",
            shift = At.Shift.AFTER))
    private void dominionsword$restoreCommandCamera(float partialTick, long finishNanoTime,
                                                     PoseStack poseStack, CallbackInfo ci) {
        CommandCameraGuard.restore();
    }
}
