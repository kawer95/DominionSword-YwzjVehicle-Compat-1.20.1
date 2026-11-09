package com.arxyt.dominionsword.ywzjvehiclecompat;

import net.minecraft.world.phys.Vec3;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class TerrainFollowingPolicyTest {
    @Test void limitsCloseMovesAndKTurnsBeforeApplyingThrottle() {
        assertEquals(0.10D, YwzjVehicleAdapter.wheelSpeedLimit(8, 0, false, false));
        assertEquals(0.065D, YwzjVehicleAdapter.wheelSpeedLimit(3, 150, false, true));
        assertEquals(0.10D, YwzjVehicleAdapter.wheelSpeedLimit(40, 100, false, false));
        assertEquals(0.10D, YwzjVehicleAdapter.wheelSpeedLimit(40, 0, true, false));
        assertEquals(0.50D, YwzjVehicleAdapter.wheelSpeedLimit(48, 0, false, false));
    }

    @Test void pulsesThrottleAndUsesTheOppositePedalToSlowDown() {
        assertEquals(1, YwzjVehicleAdapter.wheelPedal(1, 0, .10, 0));
        assertEquals(0, YwzjVehicleAdapter.wheelPedal(1, 0, .10, 2));
        assertEquals(-1, YwzjVehicleAdapter.wheelPedal(1, .15, .10, 0));
        assertEquals(1, YwzjVehicleAdapter.wheelPedal(-1, -.15, .10, 0));
        assertEquals(-1, YwzjVehicleAdapter.wheelPedal(0, .07, 0, 0));
        assertEquals(0, YwzjVehicleAdapter.wheelPedal(0, .001, 0, 0));
    }

    @Test void choosesKTurnWhenARearGoalIsInsideTheTurningCircle() {
        assertTrue(YwzjVehicleAdapter.shortReverseFits(153,9,8));
        assertTrue(YwzjVehicleAdapter.shortReverseFits(164,2.8,8));
        assertFalse(YwzjVehicleAdapter.shortReverseFits(124,3.6,8));
        assertTrue(YwzjVehicleAdapter.shortReverseFits(180,20,8));
        assertFalse(YwzjVehicleAdapter.shortReverseFits(180,30,8));
        assertFalse(YwzjVehicleAdapter.shortReverseFits(90,8,8));
    }

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

    @Test
    void usesManeuversForFinalGoalsWithoutReversingAtEveryWaypoint() {
        assertFalse(YwzjVehicleAdapter.shouldUseThreePointTurn(true, false, false, false, true));
        assertTrue(YwzjVehicleAdapter.shouldUseThreePointTurn(true, false, false, true, true));
    }

    @Test
    void pivotsTrackedVehiclesOnlyForCloseLargeTurns() {
        assertTrue(YwzjVehicleAdapter.shouldPivotTrackedVehicle(true, 90.0D, 14.0D, 18.0D, false));
        assertTrue(YwzjVehicleAdapter.shouldPivotTrackedVehicle(true, 90.0D, 26.0D, 18.0D, true));
        assertFalse(YwzjVehicleAdapter.shouldPivotTrackedVehicle(true, 42.0D, 8.0D, 18.0D, false));
        assertFalse(YwzjVehicleAdapter.shouldPivotTrackedVehicle(true, 90.0D, 40.0D, 18.0D, true));
        assertFalse(YwzjVehicleAdapter.shouldPivotTrackedVehicle(false, 90.0D, 8.0D, 18.0D, false));
    }

    @Test
    void usesPivotBeforeRouteDetourWhenTrackedVehicleHitsAnObstacle() {
        assertTrue(YwzjVehicleAdapter.shouldAttemptTrackedEscapePivot(true, true));
        assertFalse(YwzjVehicleAdapter.shouldAttemptTrackedEscapePivot(true, false));
        assertFalse(YwzjVehicleAdapter.shouldAttemptTrackedEscapePivot(false, true));
    }
}
