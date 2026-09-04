package com.arxyt.dominionsword.ywzjvehiclecompat.mixin;

import net.minecraft.client.Camera;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.gen.Invoker;

@Mixin(Camera.class)
public interface CameraAccess {
    @Invoker("setPosition")
    void dominionsword$setPosition(double x, double y, double z);

    @Invoker("setRotation")
    void dominionsword$setRotation(float yaw, float pitch);
}
