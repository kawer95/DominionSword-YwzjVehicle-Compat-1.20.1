package com.arxyt.dominionsword.ywzjvehiclecompat;

import net.minecraft.world.phys.Vec3;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class TrackedHeightIdentityTest {
    @Test void frontierKeysKeepBridgeAndRoadAsSeparateStates() {
        Vec3 anchor = new Vec3(0, 64, 0);
        assertNotEquals(YwzjVehicleAdapter.trackedPoseKey(anchor, new Vec3(4, 64, 2), 3),
                YwzjVehicleAdapter.trackedPoseKey(anchor, new Vec3(4, 72, 2), 3));
        assertNotEquals(YwzjVehicleAdapter.trackedPoseKey(anchor, new Vec3(4, 64, 2), 3),
                YwzjVehicleAdapter.trackedPoseKey(anchor, new Vec3(4, 64.125D, 2), 3));
    }

    @Test void collisionCacheKeepsExactSupportHeightAndYaw() {
        Vec3 lower = new Vec3(4, 64, 2);
        assertNotEquals(YwzjVehicleAdapter.trackedPoseCacheKey(lower, 30.0F),
                YwzjVehicleAdapter.trackedPoseCacheKey(new Vec3(4, 64.01D, 2), 30.0F));
        assertNotEquals(YwzjVehicleAdapter.trackedPoseCacheKey(lower, 30.0F),
                YwzjVehicleAdapter.trackedPoseCacheKey(lower, 30.01F));
    }

    @Test void routeNodesCannotBeConsumedFromAnotherLevel() {
        Vec3 previous = new Vec3(0, 64, 0), point = new Vec3(2, 64, 0);
        assertTrue(YwzjVehicleAdapter.hasPassedPoint(new Vec3(3, 64, 0), previous, point));
        assertFalse(YwzjVehicleAdapter.hasPassedPoint(new Vec3(3, 72, 0), previous, point));
        assertFalse(YwzjVehicleAdapter.routeEndpointMatches(new Vec3(2, 72, 0), point, 1));
        assertTrue(YwzjVehicleAdapter.routeEndpointMatches(new Vec3(2, 64.5D, 0), point, 1));
    }

    @Test void completedRouteDoesNotReuseAnOldNearbyDestination() {
        Vec3 planned=new Vec3(0,64,0);
        assertFalse(YwzjVehicleAdapter.trackedRouteMatchesTarget(planned,new Vec3(.9D,64,0)));
        assertFalse(YwzjVehicleAdapter.trackedRouteMatchesTarget(planned,new Vec3(0,72,0)));
        assertTrue(YwzjVehicleAdapter.trackedRouteMatchesTarget(planned,new Vec3(.05D,64,0)));
    }
}
