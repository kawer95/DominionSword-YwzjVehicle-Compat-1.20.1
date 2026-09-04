package com.arxyt.dominionsword.ywzjvehiclecompat;

/** Pure arbitration rule between YWZJ's mounted HUD suppression and Dominion's detached HUD. */
public final class CommandCameraHudPolicy {
    private CommandCameraHudPolicy() {
    }

    public static boolean bypassMountedHudSuppression(boolean commandCameraOwnsView) {
        return commandCameraOwnsView;
    }
}
