# 1.4.0-beta.3 — Remaining march route snapshots

Requires Dominion Sword 1.37.0-beta.3. `marchRoute` returns the vehicle's current position and remaining, unconsumed real path nodes. Pending/failed paths remain empty; reaching the end of a partial path allows the existing incremental planner to continue. Debug straight-line previews are not used as movement routes. Native driving/physics is unchanged in this increment.

# 1.4.0-beta.2 — Unified march ownership

Requires Dominion Sword 1.37.0-beta.2. Ground driving yields to the core persistent column/free task's waiting and conservative cruise-cap decisions. Native movement, collision and special flight/jump controllers remain addon-owned. No live physics or load benchmark has been performed.

# 1.4.0-beta.1 — Ground pathfinding optimization

Requires Dominion Sword 1.37.0-beta.1. Snapshot sampling, result validation and tracked pose expansion share the core server planning budget. Coarse sampling starts near the command corridor and expands if necessary. Superseded futures are cancelled; partial paths cannot append an unchecked destination. Tracked/wheeled profiles opt into experimental core shared corridors; aircraft remain on their existing controller. Waiting applies native braking while suppressing the persistent movement callback.

The native tracked controller and recovery behavior remain addon-owned, including the existing exceptional depenetration fallback. sharedGroundRoutes is off by default pending in-game validation.
