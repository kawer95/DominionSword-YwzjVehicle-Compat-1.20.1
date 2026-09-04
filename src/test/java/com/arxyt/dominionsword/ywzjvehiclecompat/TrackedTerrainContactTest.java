package com.arxyt.dominionsword.ywzjvehiclecompat;

import net.minecraft.world.phys.AABB;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class TrackedTerrainContactTest {
    @Test
    void flatGroundAndOneBlockGradeAreSupport() {
        assertTrue(YwzjVehicleAdapter.trackedTerrainContact(70.0D, 0.0D,
                new AABB(0.0D, 69.0D, 0.0D, 1.0D, 70.0D, 1.0D)));
        assertTrue(YwzjVehicleAdapter.trackedTerrainContact(70.0D, 0.0D,
                new AABB(0.0D, 70.0D, 0.0D, 1.0D, 71.0D, 1.0D)));
        // After the centre drops, the rear may still bridge the preceding upper block.
        assertTrue(YwzjVehicleAdapter.trackedTerrainContact(69.0D, 0.0D,
                new AABB(0.0D, 69.0D, 0.0D, 1.0D, 70.0D, 1.0D)));
        // Real YWZJ hulls may sit slightly below the nominal support surface on a slope.
        assertTrue(YwzjVehicleAdapter.trackedTerrainContact(69.898D, -0.102D,
                new AABB(0.0D, 70.0D, 0.0D, 1.0D, 71.0D, 1.0D)));
    }

    @Test
    void floatingAndTooTallShapesRemainObstacles() {
        assertFalse(YwzjVehicleAdapter.trackedTerrainContact(70.0D, 0.0D,
                new AABB(0.0D, 70.5D, 0.0D, 1.0D, 71.0D, 1.0D)));
        assertFalse(YwzjVehicleAdapter.trackedTerrainContact(70.0D, 0.0D,
                new AABB(0.0D, 70.0D, 0.0D, 1.0D, 72.0D, 1.0D)));
    }

    @Test
    void exposedStairAheadOfLongHullIsTerrain() {
        AABB stairTop = new AABB(0.0D, 66.0D, 0.0D, 1.0D, 67.0D, 1.0D);
        assertTrue(YwzjVehicleAdapter.trackedExposedTerrainContact(
                64.0D, 0.0D, stairTop, 2.0D, true));
    }

    @Test
    void coveredColumnIsWallNotTerrain() {
        AABB lowerWall = new AABB(0.0D, 65.0D, 0.0D, 1.0D, 66.0D, 1.0D);
        assertFalse(YwzjVehicleAdapter.trackedExposedTerrainContact(
                64.0D, 0.0D, lowerWall, 2.0D, false));
    }

    @Test
    void longHullMayBridgeMissingCentreSupport() {
        assertTrue(YwzjVehicleAdapter.trackedSupportPatternAccepts(false, true, true, false, false));
        assertTrue(YwzjVehicleAdapter.trackedSupportPatternAccepts(false, true, false, true, false));
        assertTrue(YwzjVehicleAdapter.trackedSupportPatternAccepts(true, false, false, false, false));
    }

    @Test
    void floatingHullAndSingleUnsupportedCornerAreRejected() {
        assertFalse(YwzjVehicleAdapter.trackedSupportPatternAccepts(false, false, false, false, false));
        assertFalse(YwzjVehicleAdapter.trackedSupportPatternAccepts(false, true, false, false, false));
    }
}
