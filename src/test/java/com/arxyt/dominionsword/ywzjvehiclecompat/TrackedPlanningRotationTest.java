package com.arxyt.dominionsword.ywzjvehiclecompat;

import org.joml.Quaternionf;
import org.joml.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class TrackedPlanningRotationTest {
    @Test
    void navigationFrameContainsYawButNeverPhysicalPitchOrRoll() {
        Quaternionf rotation = YwzjVehicleAdapter.trackedPlanningRotation(90.0F);
        Vector3f up = rotation.transform(new Vector3f(0.0F, 1.0F, 0.0F));
        assertEquals(0.0F, up.x, 1.0E-6F);
        assertEquals(1.0F, up.y, 1.0E-6F);
        assertEquals(0.0F, up.z, 1.0E-6F);

        Vector3f forward = rotation.transform(new Vector3f(0.0F, 0.0F, 1.0F));
        assertEquals(-1.0F, forward.x, 1.0E-6F);
        assertEquals(0.0F, forward.y, 1.0E-6F);
        assertEquals(0.0F, forward.z, 1.0E-6F);
    }
}
