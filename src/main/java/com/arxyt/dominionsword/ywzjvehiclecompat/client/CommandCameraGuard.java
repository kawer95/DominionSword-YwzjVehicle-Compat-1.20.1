package com.arxyt.dominionsword.ywzjvehiclecompat.client;

import com.arxyt.dominionsword.client.ClientSpirit;
import com.arxyt.dominionsword.ywzjvehiclecompat.mixin.CameraAccess;
import net.minecraft.client.Camera;
import net.minecraft.client.Minecraft;
import net.minecraft.world.phys.Vec3;

/** Holds one render call's vanilla command-camera transform and discards it immediately. */
public final class CommandCameraGuard {
    private static Vec3 position;
    private static float yaw;
    private static float pitch;
    private static boolean pending;

    private CommandCameraGuard() {
    }

    public static void capture(Camera camera) {
        position = camera.getPosition();
        yaw = camera.getYRot();
        pitch = camera.getXRot();
        pending = true;
    }

    public static void restore() {
        if (!pending) return;
        pending = false;
        if (!ClientSpirit.detachedCameraTransition() || position == null) return;
        Camera camera = Minecraft.getInstance().gameRenderer.getMainCamera();
        CameraAccess access = (CameraAccess) camera;
        access.dominionsword$setPosition(position.x, position.y, position.z);
        access.dominionsword$setRotation(yaw, pitch);
    }
}
