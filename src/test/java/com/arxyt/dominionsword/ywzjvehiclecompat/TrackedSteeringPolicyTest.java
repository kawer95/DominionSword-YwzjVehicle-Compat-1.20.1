package com.arxyt.dominionsword.ywzjvehiclecompat;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class TrackedSteeringPolicyTest {
    @Test
    void modestHeadingCorrectionOnTranslationSteersWhileDriving() {
        assertTrue(YwzjVehicleAdapter.trackedShouldRollingSteer(false, 12.2F));
        assertTrue(YwzjVehicleAdapter.trackedShouldRollingSteer(false, -30.0F));
    }

    @Test
    void pureRotationAndLargeTurnStillPivot() {
        assertFalse(YwzjVehicleAdapter.trackedShouldRollingSteer(true, 12.2F));
        assertFalse(YwzjVehicleAdapter.trackedShouldRollingSteer(false, 30.1F));
        assertFalse(YwzjVehicleAdapter.trackedShouldRollingSteer(false, 3.9F));
    }

    @Test
    void rearTargetUnderFifteenBlocksUsesDirectReverse() {
        assertTrue(YwzjVehicleAdapter.trackedShouldDirectShortReverse(0.0F, 180.0F, 14.99D));
        assertFalse(YwzjVehicleAdapter.trackedShouldDirectShortReverse(0.0F, 180.0F, 15.0D));
        assertFalse(YwzjVehicleAdapter.trackedShouldDirectShortReverse(0.0F, 140.0F, 10.0D));
    }
}
