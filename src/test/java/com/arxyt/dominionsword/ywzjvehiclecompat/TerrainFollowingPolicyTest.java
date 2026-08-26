package com.arxyt.dominionsword.ywzjvehiclecompat;

import net.minecraft.world.phys.Vec3;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class TerrainFollowingPolicyTest {
    @Test
    void rebasesEachProbeOntoTheLastGroundContact() {
        Vec3 probe = YwzjVehicleAdapter.terrainProbe(new Vec3(-7.0D, 90.0D, 5.0D), 61.0D);

        assertEquals(-7.0D, probe.x);
        assertEquals(61.0D, probe.y);
        assertEquals(5.0D, probe.z);
    }

    @Test
    void acceptsContinuousSlopesButNotWallSizedSteps() {
        assertTrue(YwzjVehicleAdapter.terrainStepAllowed(64.0D, 63.0D));
        assertTrue(YwzjVehicleAdapter.terrainStepAllowed(63.0D, 64.0D));
        assertFalse(YwzjVehicleAdapter.terrainStepAllowed(64.0D, 62.5D));
        assertEquals(3.15D, YwzjVehicleAdapter.terrainGridStepHeight(), 1.0E-9D);
    }

    @Test
    void acceptsSlopeAcrossWholeHullButRejectsWallInFootprint() {
        assertTrue(YwzjVehicleAdapter.terrainFootprintProfileAllowed(new double[][]{
                {72.0D, 71.0D, 70.0D},
                {72.0D, 71.0D, 70.0D},
                {72.0D, 71.0D, 70.0D}
        }));
        assertFalse(YwzjVehicleAdapter.terrainFootprintProfileAllowed(new double[][]{
                {72.0D, 72.0D, 72.0D},
                {72.0D, 76.0D, 72.0D},
                {72.0D, 72.0D, 72.0D}
        }));
    }
}
