# 1.4.0-beta.1 — Ground pathfinding optimization

Requires Dominion Sword 1.37.0-beta.1. Snapshot sampling, result validation and tracked pose expansion share the core server planning budget. Coarse sampling starts near the command corridor and expands if necessary. Superseded futures are cancelled; partial paths cannot append an unchecked destination. Tracked/wheeled profiles opt into experimental core shared corridors; aircraft remain on their existing controller. Waiting applies native braking while suppressing the persistent movement callback.

The native tracked controller and recovery behavior remain addon-owned, including the existing exceptional depenetration fallback. sharedGroundRoutes is off by default pending in-game validation.
