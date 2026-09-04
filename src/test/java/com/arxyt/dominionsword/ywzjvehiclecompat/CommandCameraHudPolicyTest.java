package com.arxyt.dominionsword.ywzjvehiclecompat;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class CommandCameraHudPolicyTest {
    @Test
    void onlyDetachedCommandCameraBypassesYwzjMountedHudSuppression() {
        assertTrue(CommandCameraHudPolicy.bypassMountedHudSuppression(true));
        assertFalse(CommandCameraHudPolicy.bypassMountedHudSuppression(false));
    }
}
