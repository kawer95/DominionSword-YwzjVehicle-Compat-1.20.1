package com.arxyt.dominionsword.ywzjvehiclecompat;

import com.arxyt.dominionsword.api.DominionControlApi;
import com.arxyt.dominionsword.api.DominionVehicleAdapter;
import com.arxyt.dominionsword.api.DominionAsyncGridPlanner;
import com.mojang.logging.LogUtils;
import net.minecraft.core.BlockPos;
import net.minecraft.core.SectionPos;
import net.minecraft.nbt.CompoundTag;
import net.minecraft.nbt.ListTag;
import net.minecraft.nbt.Tag;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.server.level.ServerPlayer;
import net.minecraft.resources.ResourceKey;
import net.minecraft.util.Mth;
import net.minecraft.world.entity.Entity;
import net.minecraft.world.entity.LivingEntity;
import net.minecraft.world.entity.Mob;
import net.minecraft.world.level.ClipContext;
import net.minecraft.world.level.Level;
import net.minecraft.world.level.block.Block;
import net.minecraft.world.level.block.Blocks;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.level.levelgen.Heightmap;
import net.minecraft.world.level.chunk.LevelChunk;
import net.minecraft.world.level.chunk.LevelChunkSection;
import net.minecraft.world.phys.AABB;
import net.minecraft.world.phys.HitResult;
import net.minecraft.world.phys.Vec3;
import net.minecraft.world.phys.shapes.VoxelShape;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.UUID;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CompletableFuture;
import org.slf4j.Logger;
import org.ywzj.vehicle.entity.vehicle.AbstractVehicle;
import org.ywzj.vehicle.entity.vehicle.FixedWingVehicle;
import org.ywzj.vehicle.entity.vehicle.RotaryWingVehicle;
import org.ywzj.vehicle.entity.vehicle.WheeledVehicle;
import org.ywzj.vehicle.vehicle.control.ControlUnit;
import org.ywzj.vehicle.vehicle.part.WeaponUnit;
import org.ywzj.vehicle.vehicle.pojo.AimContext;
import org.ywzj.vehicle.vehicle.structure.OBB;
import org.ywzj.vehicle.vehicle.structure.VehicleCubeOBB;
import org.ywzj.vehicle.vehicle.weapon.AbstractVehicleWeapon;
import org.joml.Quaternionf;
import org.joml.Vector3f;

/** Reflective bridge for Limitless Vehicle / ywzj_vehicle. */
public final class YwzjVehicleAdapter implements DominionVehicleAdapter {
    private static final Logger LOGGER = LogUtils.getLogger();
    private static final String VEHICLE_CLASS_NAME = "org.ywzj.vehicle.entity.vehicle.AbstractVehicle";
    private static final String FIXED_WING_CLASS_NAME = "org.ywzj.vehicle.entity.vehicle.FixedWingVehicle";
    private static final String ROTARY_WING_CLASS_NAME = "org.ywzj.vehicle.entity.vehicle.RotaryWingVehicle";
    private static final String WEAPON_UNIT_CLASS_NAME = "org.ywzj.vehicle.vehicle.part.WeaponUnit";
    private static final String ACTION_HELI_TAKEOFF = "ywzj_helicopter_takeoff";
    private static final String ACTION_HELI_LAND = "ywzj_helicopter_land";
    private static final String ACTION_HELI_LOW = "ywzj_helicopter_low_hover";
    private static final String ACTION_HELI_MEDIUM = "ywzj_helicopter_medium_hover";
    private static final String ACTION_HELI_HIGH = "ywzj_helicopter_high_hover";
    private static final String ACTION_HELI_HOLD_ALTITUDE = "ywzj_helicopter_hold_altitude";
    private static final String ACTION_HELI_SET_ABSOLUTE_ALTITUDE = "ywzj_helicopter_set_absolute_altitude";
    private static final String HELI_MODE = "DominionSwordYwzjHeliMode";
    private static final String HELI_HOLD_ALTITUDE = "DominionSwordYwzjHeliHoldAltitude";
    private static final String HELI_LOCKED_ALTITUDE = "DominionSwordYwzjHeliLockedAltitude";
    private static final String HELI_NAV_X = "DominionSwordYwzjHeliNavX";
    private static final String HELI_NAV_Y = "DominionSwordYwzjHeliNavY";
    private static final String HELI_NAV_Z = "DominionSwordYwzjHeliNavZ";
    private static final String HELI_LAST_CONTROL_TRACE_TICK = "DominionSwordYwzjHeliLastControlTraceTick";
    private static final String HELI_LAST_WEAPON_TRACE_TICK = "DominionSwordYwzjHeliLastWeaponTraceTick";
    private static final String HELI_BRAKE_LATCH = "DominionSwordYwzjHeliBrakeLatch";
    private static final String HELI_COMBAT_TARGET = "DominionSwordYwzjHeliCombatTarget";
    /** Active persistent flight tasks only; never retain loaded Entity instances. */
    private static final Map<ResourceKey<Level>, Set<UUID>> ACTIVE_HELICOPTERS = new ConcurrentHashMap<>();
    /** Last autonomous command for a ground chassis, replayed before its native physics tick. */
    private static final Map<ResourceKey<Level>, Map<UUID, GroundControlState>> ACTIVE_GROUND_CONTROLS = new ConcurrentHashMap<>();
    private static final Map<UUID, Long> GROUND_CONTROL_TRACE_TICKS = new ConcurrentHashMap<>();
    private static final double HELI_CLOSE_TRANSLATE_RANGE = 30.0D;
    private static final double HELI_MAX_HORIZONTAL_SPEED = 0.72D;
    private static final double HELI_POSITION_TO_SPEED_GAIN = 0.055D;
    private static final double HELI_FORWARD_SPEED_TO_PITCH_GAIN = 22.0D;
    private static final double HELI_SIDE_SPEED_TO_ROLL_GAIN = 24.0D;
    private static final double HELI_STATION_KEEPING_SPEED = 0.035D;
    private static final double HELI_LOW_ALTITUDE = 10.0D;
    private static final double HELI_MEDIUM_ALTITUDE = 20.0D;
    private static final double HELI_HIGH_ALTITUDE = 30.0D;
    private static final String HELI_CLEARANCE_SCAN_TICK = "DominionSwordYwzjHeliClearanceScanTick";
    private static final String HELI_CLEARANCE_SCAN_TARGET_X = "DominionSwordYwzjHeliClearanceTargetX";
    private static final String HELI_CLEARANCE_SCAN_TARGET_Z = "DominionSwordYwzjHeliClearanceTargetZ";
    private static final String HELI_CLEARANCE_SCAN_GROUND_Y = "DominionSwordYwzjHeliClearanceGroundY";
    private static final long HELI_CLEARANCE_SCAN_INTERVAL = 5L;
    private static final double HELI_CLEARANCE_SCAN_STEP = 2.0D;
    private static final double HELI_CLEARANCE_SCAN_RANGE = 128.0D;
    private static final String HELI_AVOID_TARGET_X = "DominionSwordYwzjHeliAvoidTargetX";
    private static final String HELI_AVOID_TARGET_Z = "DominionSwordYwzjHeliAvoidTargetZ";
    private static final String HELI_AVOID_WAYPOINT_X = "DominionSwordYwzjHeliAvoidWaypointX";
    private static final String HELI_AVOID_WAYPOINT_Z = "DominionSwordYwzjHeliAvoidWaypointZ";
    private static final String HELI_AVOID_EXPIRES = "DominionSwordYwzjHeliAvoidExpires";
    private static final long HELI_AVOID_TTL = 80L;
    private static final double[] HELI_OBSTACLE_LATERAL_SAMPLES = {-1.0D, -0.5D, 0.0D, 0.5D, 1.0D};
    private static final double[] HELI_OBSTACLE_VERTICAL_SAMPLES = {-1.0D, -0.5D, 0.0D, 0.5D, 1.0D};

    private static final double ARRIVE_RADIUS = 1.0D;
    private static final double SLOW_RADIUS = 8.0D;
    private static final double CLOSE_REVERSE_RADIUS = 25.0D;
    private static final double TURN_IN_PLACE_ANGLE = 55.0D;
    private static final double TRACKED_PIVOT_CORNER_ANGLE = 82.0D;
    private static final double TRACKED_ESCAPE_MIN_ANGLE = 28.0D;
    private static final long TRACKED_ESCAPE_PIVOT_TICKS = 50L;
    private static final double REVERSE_ANGLE = 150.0D;
    private static final double SIDE_REVERSE_ANGLE = 100.0D;
    private static final double ATTACK_RANGE = 96.0D;
    private static final double STANDOFF_RANGE = 48.0D;
    private static final double MIN_FIRE_RANGE = 6.0D;
    private static final double AIM_TOLERANCE = 8.0D;
    private static final double WEAPON_FIRE_ARC_DEGREES = 6.0D;
    private static final double PATH_STEP = 3.0D;
    private static final double PATH_SEARCH_RADIUS = 96.0D;
    private static final int PATH_MAX_ITERATIONS = 1400;
    private static final double PATH_LOOKAHEAD_MIN = 7.0D;
    private static final double PATH_LOOKAHEAD_MAX = 30.0D;
    private static final long PATH_REPLAN_COOLDOWN_TICKS = 8L;
    private static final String PATH_POINTS = "DominionSwordYwzjPathPoints";
    private static final String PATH_INDEX = "DominionSwordYwzjPathIndex";
    private static final String PATH_TARGET_X = "DominionSwordYwzjPathTargetX";
    private static final String PATH_TARGET_Z = "DominionSwordYwzjPathTargetZ";
    private static final String PATH_GENERATION = "DominionSwordYwzjPathGeneration";
    private static final String PATH_BLOCKED = "DominionSwordYwzjPathBlocked";
    private static final String SAFE_TARGET_X = "DominionSwordYwzjSafeTargetX";
    private static final String SAFE_TARGET_Y = "DominionSwordYwzjSafeTargetY";
    private static final String SAFE_TARGET_Z = "DominionSwordYwzjSafeTargetZ";
    private static final String FINAL_TARGET_X = "DominionSwordYwzjFinalTargetX";
    private static final String FINAL_TARGET_Z = "DominionSwordYwzjFinalTargetZ";
    private static final String REPLAN_AFTER = "DominionSwordYwzjReplanAfter";
    private static final String PATH_POLICY_NBT = "DominionVehiclePathfindingPolicy";
    private static final String PATH_POLICY_ALWAYS = "always";
    private static final String PATH_POLICY_DIRECT = "direct_only";
    private static final String THREE_POINT_STEP = "DominionSwordYwzjThreePointStep";
    private static final String THREE_POINT_TICKS = "DominionSwordYwzjThreePointTicks";
    private static final String THREE_POINT_STEER = "DominionSwordYwzjThreePointSteer";
    private static final String THREE_POINT_LAST_X = "DominionSwordYwzjThreePointLastX";
    private static final String THREE_POINT_LAST_Z = "DominionSwordYwzjThreePointLastZ";
    private static final String THREE_POINT_LAST_ABS_YAW = "DominionSwordYwzjThreePointLastAbsYaw";
    private static final String THREE_POINT_STUCK_TICKS = "DominionSwordYwzjThreePointStuckTicks";
    private static final String THREE_POINT_GIVE_UP_UNTIL = "DominionSwordYwzjThreePointGiveUpUntil";
    private static final int MAX_THREE_POINT_TICKS = 170;
    private static final int THREE_POINT_REVERSE_PHASE = 0;
    private static final int THREE_POINT_FORWARD_PHASE = 1;
    private static final int THREE_POINT_PHASE_STUCK_TICKS = 10;
    private static final int THREE_POINT_FORWARD_PHASE_TICKS = 18;
    private static final int THREE_POINT_GIVE_UP_STUCK_TICKS = 20;
    private static final long THREE_POINT_GIVE_UP_COOLDOWN_TICKS = 40L;
    private static final String THREE_POINT_TARGET_X = "DominionSwordYwzjThreePointTargetX";
    private static final String THREE_POINT_TARGET_Z = "DominionSwordYwzjThreePointTargetZ";
    private static final String TRACKED_ESCAPE_PIVOT_YAW = "DominionSwordYwzjTrackedEscapePivotYaw";
    private static final String TRACKED_ESCAPE_PIVOT_UNTIL = "DominionSwordYwzjTrackedEscapePivotUntil";
    private static final String CAPTURED_TARGET_X = "DominionSwordYwzjCapturedTargetX";
    private static final String CAPTURED_TARGET_Z = "DominionSwordYwzjCapturedTargetZ";
    private static final String EFFECTIVE_ARRIVE_SINCE = "DominionSwordYwzjEffectiveArriveSince";
    private static final long EFFECTIVE_ARRIVAL_TIMEOUT_TICKS = 10L;
    private static final double TARGET_SOFT_CHANGE_DISTANCE = 4.0D;
    private static final long RESERVATION_TTL_TICKS = 5L;
    /** Largest ground-height change accepted for each one-block collision sample. */
    private static final double MAX_TRAVEL_STEP_HEIGHT = 1.05D;
    private static final double FLEET_TRACK_ROAD_ACCESS_BASE = 18.0D;
    private static final double FLEET_TRACK_ROAD_LOOKAHEAD_BASE = 18.0D;
    private static final long FLEET_TRACK_MAX_AGE_TICKS = 140L;
    private static final int FLEET_TRACK_MAX_POINTS = 160;
    private static final int TRACK_ROAD_SCAN_LIMIT = 64;
    private static final long TRACK_SAFETY_CACHE_TICKS = 12L;
    private static final Map<Long, Reservation> RESERVATIONS = new ConcurrentHashMap<>();
    private static final Map<UUID, Deque<TrackPoint>> LEADER_TRACKS = new ConcurrentHashMap<>();
    private static final Map<TrackRoadKey, TrackRoadSegment> TRACK_ROADS = new ConcurrentHashMap<>();
    private static final Map<TrackSafetyKey, CachedTrackSafety> TRACK_SAFETY_CACHE = new ConcurrentHashMap<>();
    private static final Map<ClassNameKey, Boolean> CLASS_MATCH_CACHE = new ConcurrentHashMap<>();
    private static final Map<ClassNameKey, Boolean> CLASS_CONTAINS_CACHE = new ConcurrentHashMap<>();
    private static final Map<Block, Double> TERRAIN_PENALTY_CACHE = new ConcurrentHashMap<>();
    private static final Map<FieldKey, Optional<Field>> FIELD_CACHE = new ConcurrentHashMap<>();
    private static final Map<MethodKey, Optional<Method>> METHOD_CACHE = new ConcurrentHashMap<>();
    private static final Set<MethodKey> BROKEN_METHODS = ConcurrentHashMap.newKeySet();
    private static final Map<UUID, AsyncRouteBuild> ASYNC_ROUTES = new ConcurrentHashMap<>();
    /**
     * A tracked hull cannot be planned as an un-orientated point.  These searches run on the
     * server thread in small slices because each candidate uses the same main-OBB surface
     * samples as YWZJ's native physics; putting live Level access on an async worker is unsafe.
     */
    private static final Map<UUID, TrackedPoseRouteBuild> TRACKED_POSE_BUILDS = new ConcurrentHashMap<>();
    private static final Map<UUID, TrackedPoseRoute> TRACKED_POSE_ROUTES = new ConcurrentHashMap<>();
    private static final Map<UUID, TrackedRecovery> TRACKED_RECOVERIES = new ConcurrentHashMap<>();
    private static final Map<UUID, TrackedMotionWatch> TRACKED_MOTION_WATCHES = new ConcurrentHashMap<>();
    /**
     * The yaw-only navigation hull can disagree with a physically pitched chassis while that
     * chassis bridges a ditch or stair edge.  During this short window native physics remains
     * the authority instead of leaving a driveable tank on neutral controls.
     */
    private static final Map<UUID, Long> TRACKED_PROXY_DRIVE_UNTIL = new ConcurrentHashMap<>();
    private static final Map<UUID, Long> HELICOPTER_SELECTION_TRACE_TICKS = new ConcurrentHashMap<>();
    private static final int ASYNC_SNAPSHOT_CELLS_PER_TICK = 72;
    private static final String PATH_ASYNC_PENDING = "DominionSwordYwzjPathAsyncPending";
    private static final double TRACKED_POSE_CELL = 0.5D;
    private static final double TRACKED_POSE_FORWARD_STEP = 1.0D;
    /** A chassis rotation is a stop-and-pivot manoeuvre, not 0.3 blocks of free travel. */
    private static final double TRACKED_POSE_ROTATION_COST = 2.75D;
    /** Keep A* target-directed without making the first zig-zag goal state dominate route quality. */
    private static final double TRACKED_POSE_HEURISTIC_WEIGHT = 1.10D;
    // A 30 degree heading lattice is enough for the tank's pivot steering, but avoids
    // spending half of a route search on nearly-identical 15 degree poses.
    private static final int TRACKED_POSE_HEADINGS = 12;
    private static final int TRACKED_POSE_EXPANSIONS_PER_TICK = 24;
    /** Receding-horizon planner: never hold a driveable tank still for a long global search. */
    private static final int TRACKED_POSE_PARTIAL_ROUTE_TICKS = 20;
    private static final double TRACKED_POSE_PARTIAL_MIN_PROGRESS = 0.75D;
    /** Moving formation targets may drift while an incremental search is running. */
    private static final double TRACKED_POSE_TARGET_REUSE_RADIUS = 8.0D;
    /** Hard server-thread slice per vehicle.  Node count alone cannot bound OBB work. */
    private static final long TRACKED_POSE_SEARCH_BUDGET_NANOS = 2_000_000L;
    /** Route string-pulling runs synchronously after A* and therefore needs its own ceiling. */
    private static final long TRACKED_POSE_SIMPLIFY_BUDGET_NANOS = 2_000_000L;
    private static final int TRACKED_POSE_MAX_EXPANSIONS = 10000;
    private static final double TRACKED_POSE_MAX_RANGE = 48.0D;
    private static final double TRACKED_POSE_GOAL_RADIUS = 1.10D;
    private static final double TRACKED_POSE_REACH_RADIUS = 0.36D;
    private static final double TRACKED_POSE_YAW_TOLERANCE = 4.0D;
    private static final double TRACKED_POSE_DIRECT_REVERSE_RANGE = 15.0D;
    /** Translation nodes steer into modest corrections; only real turns stop and pivot. */
    private static final double TRACKED_POSE_ROLLING_STEER_MAX_YAW = 30.0D;
    // trackedBrakeControl releases at 0.025 to avoid flipping into reverse.  Pivot only at
    // that same near-zero threshold; the previous 0.045 still produced visible forward drift.
    private static final double TRACKED_POSE_PIVOT_SPEED = 0.026D;
    private static final double TRACKED_NATIVE_BRAKE_PER_TICK = 0.025D;
    private static final double TRACKED_RECOVERY_DISTANCE = 1.75D;
    private static final int TRACKED_RECOVERY_MAX_TICKS = 90;

    @Override
    public int priority() {
        return 50;
    }

    @Override
    public boolean supports(Entity vehicle) {
        return vehicle instanceof AbstractVehicle;
    }

    @Override
    public boolean selectable(Entity vehicle) {
        // Empty vehicles may still be targeted for boarding and rotary-wing craft
        // keep their dashed projected ring, but direct control requires a passenger.
        return supports(vehicle) && hasAnyPassenger(vehicle);
    }

    @Override
    public float portraitPitchDegrees(Entity vehicle) {
        // Rotary-wing models follow the native preview convention as-is. Ground
        // vehicle model roots use the opposite vertical axis and require the
        // inverse presentation pitch to expose the roof rather than the belly.
        return isRotaryWingVehicle(vehicle) ? 16.0F : -16.0F;
    }

    @Override
    public float portraitYawDegrees(Entity vehicle) {
        // Helicopters already face the correct way in the native preview axes.
        // Ground vehicle model fronts are reversed, so rotate only those by 180°.
        return isRotaryWingVehicle(vehicle) ? 35.0F : 215.0F;
    }

    @Override
    public PortraitRenderScope beginPortraitRender(Entity vehicle) {
        return MuzzleFlashPortraitScope.suppress(vehicle);
    }

    @Override
    public PortraitTransform portraitTransform(Entity vehicle) {
        // YWZJ's own VehicleWeaponSelectScreen uses a negative-Z scale followed by
        // X(180-pitch), Y(180+yaw). Opt into that renderer-native convention so the
        // portrait camera looks down from above instead of up through the chassis.
        return PortraitTransform.INVENTORY_XY;
    }

    @Override
    public AABB selectionBounds(Entity vehicle) {
        AABB fromObb = allComponentObbAabb(vehicle);
        AABB base = fromObb == null ? vehicle.getBoundingBox() : union(fromObb, vehicle.getBoundingBox());
        if (isRotaryWingVehicle(vehicle)) {
            base = base.inflate(1.0D, 0.0D, 1.0D);
            double groundY = projectedFootprintGroundY(vehicle, base);
            AABB projected = new AABB(base.minX - 0.75D, groundY + 0.02D, base.minZ - 0.75D,
                    base.maxX + 0.75D, groundY + 0.18D, base.maxZ + 0.75D);
            traceHelicopterSelection(vehicle, projected, groundY);
            return projected;
        }
        return base.inflate(0.75D, 0.25D, 0.75D);
    }

    @Override
    public AABB portraitBounds(Entity vehicle) {
        AABB components = allComponentObbAabb(vehicle);
        return components == null ? vehicle.getBoundingBox() : union(components, vehicle.getBoundingBox());
    }

    @Override
    public float portraitScaleMultiplier(Entity vehicle) {
        // Component OBB unions are deliberately conservative (rotors, barrels and
        // attachments). Recover the otherwise visible empty ring after exact 2-D fit.
        return 1.10F;
    }

    @Override
    public List<Vec3> selectionCorners(Entity vehicle) {
        if (isRotaryWingVehicle(vehicle)) return DominionVehicleAdapter.super.selectionCorners(vehicle);
        List<Vec3> corners = completeObbTopCorners(vehicle, 1.0D);
        if (!corners.isEmpty()) return corners;
        return DominionVehicleAdapter.super.selectionCorners(vehicle);
    }

    @Override
    public boolean groundProjectedSelection(Entity vehicle) {
        return isRotaryWingVehicle(vehicle);
    }

    @Override
    public HealthView health(Entity vehicle) {
        float health = readFloat(invokeNoArg(vehicle, "getHealth"), -1.0F);
        float max = readFloat(invokeNoArg(vehicle, "getMaxHealth"), -1.0F);
        return health >= 0.0F && max > 0.0F ? new HealthView(health, max) : null;
    }

    @Override
    public List<ActionView> actions(Entity vehicle) {
        if (!isRotaryWingVehicle(vehicle) || !(driver(vehicle) instanceof Mob mob)) return List.of();
        String mode = mob.getPersistentData().getString(HELI_MODE);
        if (!isHelicopterFlying(vehicle) || "LANDING".equals(mode) || "LANDED".equals(mode) || mode.isBlank()) {
            return List.of(new ActionView(ACTION_HELI_TAKEOFF, "起飞"));
        }
        return List.of(
                new ActionView(ACTION_HELI_LAND, "降落"),
                new ActionView(ACTION_HELI_LOW, "低空悬停（10格）"),
                new ActionView(ACTION_HELI_MEDIUM, "中低空悬停（20格）"),
                new ActionView(ACTION_HELI_HIGH, "高空悬停（30格）"),
                ActionView.toggle(ACTION_HELI_HOLD_ALTITUDE, "维持高度", mob.getPersistentData().getBoolean(HELI_HOLD_ALTITUDE)),
                ActionView.numberInput(ACTION_HELI_SET_ABSOLUTE_ALTITUDE, "手动设置绝对高度")
        );
    }

    @Override
    public boolean performAction(ServerPlayer player, Entity vehicle, String actionId) {
        if (!isRotaryWingVehicle(vehicle) || !(driver(vehicle) instanceof Mob mob)) return false;
        if (ACTION_HELI_TAKEOFF.equals(actionId)) {
            setHelicopterTask(mob, vehicle, "TAKEOFF");
            return true;
        }
        if (ACTION_HELI_LAND.equals(actionId)) {
            setHelicopterTask(mob, vehicle, "LANDING");
            return true;
        }
        if (ACTION_HELI_LOW.equals(actionId)) {
            setHelicopterTask(mob, vehicle, "LOW_HOVER");
            return true;
        }
        if (ACTION_HELI_MEDIUM.equals(actionId)) {
            setHelicopterTask(mob, vehicle, "MEDIUM_HOVER");
            return true;
        }
        if (ACTION_HELI_HIGH.equals(actionId)) {
            setHelicopterTask(mob, vehicle, "HIGH_HOVER");
            return true;
        }
        if (ACTION_HELI_HOLD_ALTITUDE.equals(actionId)) {
            boolean hold = !mob.getPersistentData().getBoolean(HELI_HOLD_ALTITUDE);
            mob.getPersistentData().putBoolean(HELI_HOLD_ALTITUDE, hold);
            if (hold) mob.getPersistentData().putDouble(HELI_LOCKED_ALTITUDE, vehicle.getY());
            else mob.getPersistentData().remove(HELI_LOCKED_ALTITUDE);
            return true;
        }
        return false;
    }

    @Override
    public boolean performAction(ServerPlayer player, Entity vehicle, String actionId, String value) {
        if (ACTION_HELI_SET_ABSOLUTE_ALTITUDE.equals(actionId) && isRotaryWingVehicle(vehicle) && driver(vehicle) instanceof Mob mob) {
            try {
                int altitude = Mth.clamp(Integer.parseInt(value), -64, 500);
                mob.getPersistentData().putBoolean(HELI_HOLD_ALTITUDE, true);
                mob.getPersistentData().putDouble(HELI_LOCKED_ALTITUDE, altitude);
                return true;
            } catch (NumberFormatException ignored) {
                return false;
            }
        }
        return performAction(player, vehicle, actionId);
    }

    @Override
    public List<SeatView> seats(Entity vehicle) {
        List<?> seats = asList(readMember(vehicle, "seats"));
        if (seats == null || !(vehicle.level() instanceof ServerLevel level)) return List.of();
        List<SeatView> result = new ArrayList<>();
        for (Object seat : seats) {
            int index = readInt(readMember(seat, "seatIndex"), result.size());
            int passengerId = readInt(readMember(seat, "passengerId"), -1);
            Object partUnit = readMember(seat, "partUnit");
            Entity passenger = passengerId >= 0 ? level.getEntity(passengerId) : null;
            result.add(new SeatView(index, seatType(index, partUnit), passenger));
        }
        result.sort(Comparator.comparingInt(SeatView::index));
        return result;
    }

    @Override
    public boolean hasDriver(Entity vehicle) {
        return driver(vehicle) != null;
    }

    @Override
    public boolean release(ServerPlayer player, Entity vehicle) {
        LivingEntity pilot = driver(vehicle);
        if (pilot instanceof Mob mob) mob.getPersistentData().remove(HELI_COMBAT_TARGET);
        stopVehicle(vehicle);
        return true;
    }

    @Override
    public boolean board(ServerPlayer player, Mob unit, Entity vehicle, int seat, boolean force) {
        if (!supports(vehicle) || unit == null || seat < 0) return false;
        SeatView requested = seats(vehicle).stream().filter(view -> view.index() == seat).findFirst().orElse(null);
        if (requested == null) return false;
        Entity occupant = requested.passenger();
        if (occupant != null && occupant != unit) {
            if (!force) return false;
            com.arxyt.dominionsword.api.VehicleDismounts.dismount(vehicle, occupant);
        }

        if (unit.getVehicle() != vehicle && !unit.startRiding(vehicle, true)) return false;
        Object changed = invoke(vehicle, "changeSeat", new Class<?>[]{LivingEntity.class, int.class}, unit, seat);
        if (seat == 0) {
            AbstractVehicle ywzjVehicle = (AbstractVehicle) vehicle;
            ywzjVehicle.toggleEngine(Boolean.TRUE);
            ywzjVehicle.controlUnit.setOperator(unit);
        }
        return unit.getVehicle() == vehicle && (Boolean.TRUE.equals(changed) || seatPassenger(vehicle, seat) == unit);
    }

    @Override
    public Vec3 boardingPosition(Mob unit, Entity vehicle) {
        AABB box = selectionBounds(vehicle).inflate(1.25D, 0.0D, 1.25D);
        double x = Mth.clamp(unit.getX(), box.minX, box.maxX);
        double z = Mth.clamp(unit.getZ(), box.minZ, box.maxZ);
        return new Vec3(x, vehicle.getY(), z);
    }

    @Override
    public boolean canBoardFrom(Mob unit, Entity vehicle) {
        AABB box = selectionBounds(vehicle).inflate(1.35D, 0.75D, 1.35D);
        return unit.getBoundingBox().inflate(0.45D).intersects(box)
                || unit.distanceToSqr(boardingPosition(unit, vehicle)) <= 6.25D;
    }

    @Override
    public boolean dismount(ServerPlayer player, Entity vehicle, int seat) {
        Entity passenger = seatPassenger(vehicle, seat);
        if (passenger == null) return false;
        return com.arxyt.dominionsword.api.VehicleDismounts.dismount(vehicle, passenger);
    }

    @Override
    public void prepareMoveRoute(ServerPlayer player, Entity vehicle, Vec3 target) {
        if (!supports(vehicle) || target == null || isAirVehicle(vehicle)) return;
        prepareRoute(player, vehicle, target, Set.of(vehicle.getUUID()));
    }

    @Override
    public List<Vec3> plannedMoveRoute(ServerPlayer player, Entity vehicle, Vec3 target) {
        if (!supports(vehicle) || target == null) return List.of();
        if (isRotaryWingVehicle(vehicle)) return List.of(vehicle.position(), target);
        List<Vec3> route = storedRoute(vehicle, target);
        return route.size() >= 2 ? route : List.of(vehicle.position(), target);
    }

    @Override
    public void prepareFleetMoveRoutes(ServerPlayer player, List<Entity> vehicles, List<Vec3> targets) {
        int count = Math.min(vehicles.size(), targets.size());
        for (int i = 0; i < count; i++) {
            Entity vehicle = vehicles.get(i);
            if (isAirVehicle(vehicle)) continue;
            prepareRoute(player, vehicle, targets.get(i), vehicle == null ? Set.of() : Set.of(vehicle.getUUID()));
        }
    }

    @Override
    public List<List<Vec3>> plannedFleetMoveRoutes(ServerPlayer player, List<Entity> vehicles, List<Vec3> targets) {
        List<List<Vec3>> routes = new ArrayList<>();
        int count = Math.min(vehicles.size(), targets.size());
        for (int i = 0; i < count; i++) {
            Entity vehicle = vehicles.get(i);
            Vec3 target = targets.get(i);
            if (!supports(vehicle) || target == null) continue;
            List<Vec3> route = storedRoute(vehicle, target);
            routes.add(route.size() >= 2 ? route : List.of(vehicle.position(), target));
        }
        return routes;
    }

    @Override
    public boolean move(ServerPlayer player, Entity vehicle, Vec3 target) {
        try (LagTrace ignoredTrace = LagTrace.start("ywzj.unit.move", "vehicle=" + (vehicle == null ? "null" : vehicle.getId()))) {
            if (!supports(vehicle) || target == null) return false;
            if (isRotaryWingVehicle(vehicle)) {
                LivingEntity pilot = driver(vehicle);
                if (pilot instanceof Mob mob) mob.getPersistentData().remove(HELI_COMBAT_TARGET);
                return moveHelicopter(vehicle, target);
            }
            if (isAirVehicle(vehicle)) return false;
            LivingEntity driver = driver(vehicle);
            LagTrace.mark("resolve_driver");
            if (driver == null) {
                stopVehicle(vehicle);
                return false;
            }
            ((AbstractVehicle) vehicle).toggleEngine(Boolean.TRUE);
            LagTrace.mark("engine");

            VehicleShape shape = VehicleShape.from(vehicle);
            LagTrace.mark("shape");
            ensureRoute(player, vehicle, target, shape);
            LagTrace.mark("ensure_route");
            // Never feed a tracked chassis back into the old point/AABB route follower.
            // Its route states carry the hull heading, so a node means "turn here, then
            // advance" rather than merely "put the entity centre here".
            if (isTrackedVehicle(vehicle)) {
                return moveTrackedPoseRoute(player, vehicle, target, activeSafeTarget(vehicle, target), shape);
            }
            Vec3 safeTarget = activeSafeTarget(vehicle, target);
            if (hasCapturedFinalTarget(vehicle, safeTarget, shape)) {
                stopVehicle(vehicle);
                LagTrace.mark("captured_hold");
                return true;
            }
            Vec3 driveTarget = navigationTarget(player, vehicle, safeTarget, shape);
            LagTrace.mark("navigation_target");
            if (driveTarget == null) {
                stopVehicle(vehicle);
                return true;
            }

            Vec3 flat = new Vec3(driveTarget.x - vehicle.getX(), 0.0D, driveTarget.z - vehicle.getZ());
            double distance = flat.length();
            if (flatDistance(vehicle.position(), safeTarget) <= ARRIVE_RADIUS) {
                stopVehicle(vehicle);
                return true;
            }

            float desiredYaw = yawTo(flat);
            float yawDiff = Mth.wrapDegrees(desiredYaw - vehicle.getYRot());
            double absYaw = Math.abs(yawDiff);
            double speed = horizontalSpeed(vehicle);
            if (vehicle.getPersistentData().getBoolean(PATH_ASYNC_PENDING) && isTrackedVehicle(vehicle)) {
                // Keep the hull at the snapshot origin while the coarse route grid is built.
                // Letting it drive its 6-10 block "temporary" target changed the vehicle's
                // position underneath the pending snapshot.  When the result arrived, its first
                // waypoint could be behind the tank, producing an inexplicable 180-degree turn
                // into the house it had just passed.  Tracked vehicles can use that time to
                // rotate in place toward the destination instead, then start from the exact
                // position the collision grid actually evaluated.
                boolean turn = absYaw > 3.0D;
                boolean steerRight = turn && yawDiff > 0.0F;
                boolean steerLeft = turn && yawDiff < 0.0F;
                applyControl(vehicle, false, false, steerRight, steerLeft, !turn);
                pathDebug(vehicle, turn ? "ASYNC_TRACK_PIVOT" : "ASYNC_TRACK_HOLD",
                        "target=%s safe=%s yawDiff=%.1f absYaw=%.1f keys=%d",
                        fmt(driveTarget), fmt(safeTarget), yawDiff, absYaw,
                        (int) packControl(false, false, steerRight, steerLeft, !turn));
                return true;
            }
            boolean finalTarget = flatDistance(driveTarget, safeTarget) <= 1.25D;
            double captureDistance = finalCaptureDistance(shape, speed);
            if (finalTarget && shouldHardBrakeNearFinal(vehicle, safeTarget, distance, speed, captureDistance)) {
                if (hasPassedFinalTarget(vehicle, safeTarget) && distance <= captureDistance + 0.50D) {
                    captureFinalTarget(vehicle, safeTarget);
                    pathDebug(vehicle, "FINAL_OVERSHOOT_CAPTURE", "target=%s pos=%s dist=%.2f speed=%.3f", fmt(safeTarget), fmt(vehicle.position()), distance, speed);
                }
                stopVehicle(vehicle);
                pathDebug(vehicle, "FINAL_HARD_BRAKE", "target=%s pos=%s dist=%.2f speed=%.3f capture=%.2f", fmt(safeTarget), fmt(vehicle.position()), distance, speed, captureDistance);
                return true;
            }
            if (finalTarget && shouldCaptureFinalTarget(vehicle, safeTarget, distance, speed, shape)) {
                captureFinalTarget(vehicle, safeTarget);
                stopVehicle(vehicle);
                pathDebug(vehicle, "FINAL_CAPTURE_STOP", "target=%s pos=%s dist=%.2f speed=%.3f capture=%.2f", fmt(safeTarget), fmt(vehicle.position()), distance, speed, captureDistance);
                return true;
            }
            float escapePivotYaw = trackedEscapePivotYaw(vehicle, driveTarget, shape);
            if (Float.isFinite(escapePivotYaw)) {
                float escapeYawDiff = Mth.wrapDegrees(escapePivotYaw - vehicle.getYRot());
                boolean steerRight = escapeYawDiff > 0.0F;
                boolean steerLeft = escapeYawDiff < 0.0F;
                applyControl(vehicle, false, false, steerRight, steerLeft, false);
                pathDebug(vehicle, "TRACK_ESCAPE_PIVOT", "target=%s safe=%s targetYaw=%.1f yawDiff=%.1f collision=%s keys=%d",
                        fmt(driveTarget), fmt(safeTarget), escapePivotYaw, escapeYawDiff, vehicle.horizontalCollision,
                        (int) packControl(false, false, steerRight, steerLeft, false));
                LagTrace.mark("track_escape_pivot");
                return true;
            }
            boolean trackedPivot = shouldPivotTrackedVehicle(isTrackedVehicle(vehicle), absYaw, distance,
                    trackedPivotRange(shape), !finalTarget);
            if (trackedPivot) {
                clearThreePointState(vehicle);
                boolean steerRight = yawDiff > 0.0F;
                boolean steerLeft = yawDiff < 0.0F;
                // TrackedVehicle turns on the spot when only left/right is pressed.  Do not
                // combine it with forward/backward: a tight corner must be aligned before
                // the hull enters the narrow corridor.
                applyControl(vehicle, false, false, steerRight, steerLeft, false);
                pathDebug(vehicle, "TRACK_PIVOT", "target=%s safe=%s yawDiff=%.1f absYaw=%.1f dist=%.2f range=%.2f keys=%d",
                        fmt(driveTarget), fmt(safeTarget), yawDiff, absYaw, distance, trackedPivotRange(shape),
                        (int) packControl(false, false, steerRight, steerLeft, false));
                LagTrace.mark("track_pivot");
                return true;
            }
            DriveDecision driveDecision = decideDriveMode(vehicle, yawDiff, absYaw, distance, speed, shape, finalTarget);
            short keys;
            if (driveDecision.mode() == DriveMode.REVERSE_SHORT) {
                clearThreePointState(vehicle);
                keys = shortReverseKeys(yawDiff, absYaw);
                applyPackedControl(vehicle, keys);
                rememberFutureFootprint(vehicle, driveTarget, shape);
                pathDebug(vehicle, "SHORT_REVERSE", "target=%s safe=%s yawDiff=%.1f dist=%.2f keys=%d", fmt(driveTarget), fmt(safeTarget), yawDiff, distance, (int) keys);
                return true;
            }
            if (driveDecision.mode() == DriveMode.THREE_POINT) {
                keys = threePointKeys(vehicle, yawDiff, absYaw, distance, speed, shape, driveTarget);
                if (keys != Short.MIN_VALUE) {
                    applyPackedControl(vehicle, keys);
                    rememberFutureFootprint(vehicle, driveTarget, shape);
                    pathDebug(vehicle, "THREE_POINT", "target=%s safe=%s yawDiff=%.1f speed=%.3f keys=%d", fmt(driveTarget), fmt(safeTarget), yawDiff, speed, (int) keys);
                    return true;
                }
            } else {
                clearThreePointState(vehicle);
            }
            boolean closeBehind = driveDecision.mode() == DriveMode.TURN_AROUND;
            boolean slow = distance <= SLOW_RADIUS || absYaw >= TURN_IN_PLACE_ANGLE;
            double routeLimit = routeCurvatureSpeedLimit(vehicle, driveTarget, speed);
            boolean brake = (finalTarget && shouldBrake(vehicle, distance, speed, absYaw)) || speed > routeLimit || shouldYield(vehicle, driveTarget, shape);
            // A route point is a forward steering instruction. Target yaw alone says nothing
            // about whether the car is physically boxed in.
            boolean forward = !brake && !closeBehind;
            boolean backward = !brake && closeBehind;
            boolean turn = absYaw > (slow ? 4.0D : 8.0D);
            if (finalTarget && !brake && distance <= Math.max(ARRIVE_RADIUS + 1.25D, captureDistance)) {
                brake = true;
                forward = false;
                backward = false;
            }

            if (brake) {
                Vec3 velocity = vehicle.getDeltaMovement().multiply(1.0D, 0.0D, 1.0D);
                Vec3 look = vehicle.getLookAngle().multiply(1.0D, 0.0D, 1.0D);
                double signed = velocity.lengthSqr() > 1.0E-5D && look.lengthSqr() > 1.0E-5D ? velocity.normalize().dot(look.normalize()) : 1.0D;
                boolean pulse = brakePulse(vehicle, distance, speed);
                forward = pulse && signed < -0.15D;
                backward = pulse && signed >= -0.15D;
            }

            boolean reverseSteer = false;

            boolean steerRight = turn && (reverseSteer ? yawDiff < 0.0F : yawDiff > 0.0F);
            boolean steerLeft = turn && (reverseSteer ? yawDiff > 0.0F : yawDiff < 0.0F);
            double turnRadius = estimatedTurnRadius(vehicle, shape);
            boolean activeThreePoint = vehicle.getPersistentData().contains(THREE_POINT_STEP);
            boolean cannotArc = cannotArcToTarget(vehicle, absYaw, distance, shape);
            pathDebug(vehicle, "DECISION", "mode=%s final=%s activeThreePoint=%s target=%s safe=%s yawDiff=%.1f absYaw=%.1f dist=%.2f speed=%.3f turnRadius=%.2f shortReverse=%s cannotArc=%s brake=%s forward=%s backward=%s turn=%s keys=%d",
                    driveDecision.mode(), finalTarget, activeThreePoint, fmt(driveTarget), fmt(safeTarget), yawDiff, absYaw, distance, speed, turnRadius,
                    isShortReverseTarget(absYaw, distance, shape), cannotArc, brake, forward, backward, turn,
                    (int) packControl(forward, backward, steerRight, steerLeft, brake));
            LagTrace.mark("drive_decision");
            applyControl(vehicle, forward, backward, steerRight, steerLeft, brake);
            LagTrace.mark("apply_control");
            rememberFutureFootprint(vehicle, driveTarget, shape);
            return true;
        }
    }

    @Override
    public boolean moveFleet(ServerPlayer player, List<Entity> vehicles, List<Vec3> targets) {
        try (LagTrace ignoredTrace = LagTrace.start("ywzj.fleet.move", "vehicles=" + vehicles.size() + " targets=" + targets.size())) {
        boolean any = false;
        int count = Math.min(vehicles.size(), targets.size());
        List<Entity> groundVehicles = new ArrayList<>();
        List<Vec3> groundTargets = new ArrayList<>();
        for (int i = 0; i < count; i++) {
            Entity vehicle = vehicles.get(i);
            Vec3 target = targets.get(i);
            if (isRotaryWingVehicle(vehicle)) any |= move(player, vehicle, target);
            else {
                groundVehicles.add(vehicle);
                groundTargets.add(target);
            }
        }
        if (groundVehicles.isEmpty()) return any;
        vehicles = groundVehicles;
        targets = groundTargets;
        targets = assignedFleetTargets(vehicles, targets);
        LagTrace.mark("assign");
        List<FleetMove> moves = fleetMoves(vehicles, targets);
        LagTrace.mark("fleet_moves");
        List<Vec3> commandTargets = trackRoadFleetTargets(moves);
        LagTrace.mark("track_targets");
        prepareFleetMoveRoutes(player, vehicles, commandTargets);
        LagTrace.mark("prepare_routes");
        for (FleetMove move : moves) any |= move(player, move.vehicle(), commandTargets.get(move.index()));
        LagTrace.mark("drive_units");
        return any;
        }
    }

    @Override
    public boolean attack(ServerPlayer player, Entity vehicle, LivingEntity target) {
        if (target == null || !target.isAlive()) {
            stopVehicle(vehicle);
            return false;
        }
        if (!supports(vehicle)) return false;
        if (isRotaryWingVehicle(vehicle)) {
            Vec3 targetCenter = target.getBoundingBox().getCenter();
            LivingEntity pilot = driver(vehicle);
            if (pilot instanceof Mob mob) mob.getPersistentData().putUUID(HELI_COMBAT_TARGET, target.getUUID());
            boolean visible = hasClearShot(vehicle, target);
            Vec3 movementTarget = helicopterCombatMovementTarget(vehicle, targetCenter, visible);
            // Visibility controls pursuit/fire, not body orientation. An assigned
            // air target remains the facing target even while temporarily occluded.
            boolean moving = moveHelicopter(vehicle, movementTarget, targetCenter);
            return fireAllWeapons(vehicle, target) || moving;
        }
        if (isAirVehicle(vehicle)) return false;
        double distanceSqr = vehicle.distanceToSqr(target);
        boolean visible = hasClearShot(vehicle, target);
        boolean canStandAndFire = visible && distanceSqr <= ATTACK_RANGE * ATTACK_RANGE;
        boolean acted = false;
        if (canStandAndFire && distanceSqr >= MIN_FIRE_RANGE * MIN_FIRE_RANGE && distanceSqr <= STANDOFF_RANGE * STANDOFF_RANGE) {
            stopVehicle(vehicle);
            acted = true;
        } else if (!canStandAndFire || distanceSqr > STANDOFF_RANGE * STANDOFF_RANGE) {
            acted = move(player, vehicle, target.position());
        } else {
            stopVehicle(vehicle);
            acted = true;
        }
        return (canStandAndFire && fireAllWeapons(vehicle, target)) || acted;
    }

    /** Runs persistent helicopter tasks after selection is released. */
    public void tickHelicopterAutopilot(net.minecraft.server.MinecraftServer server) {
        if (server == null) return;
        ACTIVE_HELICOPTERS.entrySet().removeIf(entry -> {
            ServerLevel level = server.getLevel(entry.getKey());
            if (level == null) return true;
            entry.getValue().removeIf(id -> {
                Entity vehicle = level.getEntity(id);
                if (vehicle == null || !vehicle.isAlive() || !isRotaryWingVehicle(vehicle)) return true;
                LivingEntity pilot = driver(vehicle);
                if (!(pilot instanceof Mob mob)) {
                    if (isHelicopterFlying(vehicle)) stopHelicopter(vehicle);
                    return true;
                }
                String mode = mob.getPersistentData().getString(HELI_MODE);
                if (mode.isBlank() || "LANDED".equals(mode)) return true;
                Vec3 facingTarget = null;
                if (mob.getPersistentData().hasUUID(HELI_COMBAT_TARGET)) {
                    Entity combatTarget = level.getEntity(mob.getPersistentData().getUUID(HELI_COMBAT_TARGET));
                    if (combatTarget instanceof LivingEntity living && living.isAlive()) facingTarget = living.getBoundingBox().getCenter();
                    else mob.getPersistentData().remove(HELI_COMBAT_TARGET);
                }
                moveHelicopter(vehicle, helicopterTaskTarget(mob, vehicle), facingTarget);
                return false;
            });
            return entry.getValue().isEmpty();
        });
    }

    /**
     * Native wheeled and tracked implementations consume their control flags in
     * their exact physics tick. Dominion's route planner deliberately runs less
     * often, so replay the latest command for every autonomous ground chassis.
     */
    public void tickGroundControl(net.minecraft.server.MinecraftServer server) {
        if (server == null) return;
        ACTIVE_GROUND_CONTROLS.entrySet().removeIf(entry -> {
            ServerLevel level = server.getLevel(entry.getKey());
            if (level == null) return true;
            Map<UUID, GroundControlState> controls = entry.getValue();
            controls.entrySet().removeIf(controlEntry -> {
                Entity vehicle = level.getEntity(controlEntry.getKey());
                if (!(vehicle instanceof AbstractVehicle ywzjVehicle) || !isPersistentGroundChassis(vehicle) || !vehicle.isAlive()) return true;
                // Never synthesize input for a player-operated vehicle.
                if (!(driver(vehicle) instanceof Mob)) return true;
                GroundControlState control = controlEntry.getValue();
                boolean alignedTrackedWeapons = false;
                if (isTrackedVehicle(vehicle)) {
                    // A Dominion move callback is intentionally sparse.  Re-evaluate the
                    // current short pose node here, at the same 20 Hz that native tracked
                    // physics consumes the keys.  This is what prevents a 0.5 block/tick
                    // tank from crossing a waypoint (or the final target) between callbacks.
                    TrackedPoseRouteBuild build = TRACKED_POSE_BUILDS.get(vehicle.getUUID());
                    if (build != null) advanceTrackedPoseRoute(null, vehicle, build);
                    TrackedPoseRoute route = TRACKED_POSE_ROUTES.get(vehicle.getUUID());
                    TrackedRecovery recovery = TRACKED_RECOVERIES.get(vehicle.getUUID());
                    if (recovery == null && isTrackedPhysicallyStuck(vehicle)) {
                        recovery = beginTrackedRecovery(vehicle, route == null ? trackedStoredTarget(vehicle) : route.target);
                    }
                    control = recovery != null
                            ? trackedRecoveryControl(vehicle, recovery)
                            : route == null && trackedProxyDriveActive(vehicle)
                            ? trackedProxyConflictControl(vehicle, trackedStoredTarget(vehicle))
                            : route == null ? trackedBrakeControl(vehicle) : trackedPoseControl(vehicle, route);
                    controls.put(vehicle.getUUID(), control);
                    traceTrackedPoseTick(vehicle, ywzjVehicle, route, recovery, control);
                    watchTrackedMotion(vehicle, ywzjVehicle, route, recovery, control);
                    if (route != null || build != null) {
                        alignMainWeaponsToYaw(vehicle, trackedWeaponAimYaw(vehicle, route, build, null), true);
                        alignedTrackedWeapons = true;
                    }
                }
                writeControl(ywzjVehicle, control.forward(), control.backward(), control.right(), control.left(), control.brake());
                if (!alignedTrackedWeapons) alignMainWeaponsForward(vehicle, control.forward());
                traceGroundControl(vehicle, ywzjVehicle, control);
                return false;
            });
            return controls.isEmpty();
        });
    }

    public void onEntityLoaded(Entity entity) {
        Entity vehicle = isRotaryWingVehicle(entity) ? entity : entity instanceof Mob mob ? mob.getVehicle() : null;
        if (vehicle == null || !isRotaryWingVehicle(vehicle) || !(driver(vehicle) instanceof Mob pilot)) return;
        String mode = pilot.getPersistentData().getString(HELI_MODE);
        if (!mode.isBlank() && !"LANDED".equals(mode)) registerActiveHelicopter(vehicle);
    }

    public void onEntityUnloaded(Entity entity) {
        Entity vehicle = isRotaryWingVehicle(entity) ? entity : entity instanceof Mob mob ? mob.getVehicle() : null;
        if (vehicle != null) unregisterActiveHelicopter(vehicle);
        unregisterActiveGroundControl(entity);
    }

    private static boolean moveHelicopter(Entity vehicle, Vec3 target) {
        return moveHelicopter(vehicle, target, null);
    }

    private static boolean moveHelicopter(Entity vehicle, Vec3 target, Vec3 facingTarget) {
        LivingEntity pilot = driver(vehicle);
        if (!(pilot instanceof Mob mob)) {
            stopHelicopter(vehicle);
            return false;
        }
        CompoundTag task = mob.getPersistentData();
        registerActiveHelicopter(vehicle);
        String mode = task.getString(HELI_MODE);
        if (mode.isBlank() || "LANDED".equals(mode)) {
            /*
             * This method is reached by an actual movement/attack command; the
             * background autopilot deliberately skips blank/LANDED tasks. A grounded
             * helicopter therefore treats the command as take off first and keeps
             * the requested destination for LOW_HOVER after clearing the ground.
             * Manual takeoff still stores the current position in performAction.
             */
            mode = vehicle.onGround() ? "TAKEOFF" : "LOW_HOVER";
            task.putString(HELI_MODE, mode);
            if ("TAKEOFF".equals(mode)) {
                task.remove(HELI_HOLD_ALTITUDE);
                task.remove(HELI_LOCKED_ALTITUDE);
            }
        }
        if (!"LANDING".equals(mode)) {
            task.putDouble(HELI_NAV_X, target.x);
            task.putDouble(HELI_NAV_Y, target.y);
            task.putDouble(HELI_NAV_Z, target.z);
        } else {
            target = helicopterTaskTarget(mob, vehicle);
        }
        ((AbstractVehicle) vehicle).toggleEngine(Boolean.TRUE);

        boolean takeoffMode = "TAKEOFF".equals(mode);
        double terrainY = takeoffMode
                ? groundProjectionY(vehicle, vehicle.getX(), vehicle.getZ())
                : groundProjectionY(vehicle, target.x, target.z);
        double landingY = helicopterLandingOriginY(vehicle, terrainY);
        double desiredY;
        if (task.getBoolean(HELI_HOLD_ALTITUDE) && task.contains(HELI_LOCKED_ALTITUDE)) {
            desiredY = boundedAbsoluteHelicopterAltitude(vehicle, task.getDouble(HELI_LOCKED_ALTITUDE));
        } else if ("LANDING".equals(mode)) {
            desiredY = landingY;
        } else if (takeoffMode) {
            desiredY = terrainY + HELI_LOW_ALTITUDE;
        } else {
            desiredY = flightClearanceGroundY(vehicle, target) + helicopterCruiseAltitude(mode);
        }
        boolean lockedAltitude = task.getBoolean(HELI_HOLD_ALTITUDE) && task.contains(HELI_LOCKED_ALTITUDE);
        Vec3 navigationTarget = lockedAltitude && !"LANDING".equals(mode)
                ? helicopterObstacleAvoidanceTarget(vehicle, target) : target;
        Vec3 flat = new Vec3(navigationTarget.x - vehicle.getX(), 0.0D, navigationTarget.z - vehicle.getZ());
        double distance = flat.length();
        float currentYaw = vehicle.getYRot();
        Vec3 facingDelta = facingTarget == null ? Vec3.ZERO
                : facingTarget.subtract(vehicle.position()).multiply(1.0D, 0.0D, 1.0D);
        float desiredYaw = facingDelta.lengthSqr() > 1.0E-6D
                ? yawTo(facingDelta)
                : distance <= HELI_CLOSE_TRANSLATE_RANGE ? currentYaw : yawTo(flat);
        double yawRadians = Math.toRadians(currentYaw);
        Vec3 forward = new Vec3(-Math.sin(yawRadians), 0.0D, Math.cos(yawRadians));
        Vec3 right = new Vec3(Math.cos(yawRadians), 0.0D, Math.sin(yawRadians));
        double localForward = flat.dot(forward);
        double localRight = flat.dot(right);
        double horizontalVelocity = horizontalSpeed(vehicle);
        boolean landing = "LANDING".equals(mode);
        boolean takingOff = "TAKEOFF".equals(mode);
        if (takingOff && vehicle.getY() >= terrainY + 4.0D) {
            mode = "LOW_HOVER";
            task.putString(HELI_MODE, mode);
            takingOff = false;
        }
        boolean reachedHorizontal = distance <= 1.25D;
        boolean reachedVertical = Math.abs(desiredY - vehicle.getY()) <= (landing ? 0.65D : 1.0D);
        /*
         * Landing is complete when the airframe has physically touched down. Do not
         * require it to remain within the old 1.25-block horizontal navigation
         * target: a small amount of drift during descent otherwise leaves the task
         * in LANDING forever, and the next tick turns the engine and collective
         * back on, producing the characteristic ground hopping.
         */
        double touchdownTerrainY = groundProjectionY(vehicle, vehicle.getX(), vehicle.getZ());
        if (landing
                && (vehicle.onGround() || helicopterBottomY(vehicle) <= touchdownTerrainY + 0.18D)
                && Math.abs(vehicle.getDeltaMovement().y) <= 0.08D) {
            completeHelicopterLanding(vehicle, mob);
            return true;
        }

        Vec3 horizontalMotion = vehicle.getDeltaMovement().multiply(1.0D, 0.0D, 1.0D);
        double currentForwardSpeed = horizontalMotion.dot(forward);
        double currentRightSpeed = horizontalMotion.dot(right);
        double desiredForwardSpeed = 0.0D;
        double desiredRightSpeed = 0.0D;
        float desiredPitch;
        float desiredRoll = 0.0F;
        String horizontalPhase;
        if (takingOff) {
            desiredPitch = 0.0F;
            horizontalPhase = "TAKEOFF_LEVEL";
        } else if (distance <= HELI_CLOSE_TRANSLATE_RANGE) {
            /*
             * Position -> desired velocity -> desired attitude.
             * Native hover only levels the aircraft; it does not cancel horizontal
             * momentum. The velocity term therefore commands counter-pitch/bank
             * before arrival and keeps braking after the aircraft crosses the point.
             */
            desiredForwardSpeed = Mth.clamp(localForward * HELI_POSITION_TO_SPEED_GAIN,
                    -HELI_MAX_HORIZONTAL_SPEED, HELI_MAX_HORIZONTAL_SPEED);
            desiredRightSpeed = Mth.clamp(localRight * HELI_POSITION_TO_SPEED_GAIN,
                    -HELI_MAX_HORIZONTAL_SPEED, HELI_MAX_HORIZONTAL_SPEED);
            desiredPitch = (float) Mth.clamp(
                    (desiredForwardSpeed - currentForwardSpeed) * HELI_FORWARD_SPEED_TO_PITCH_GAIN,
                    -14.0D, 14.0D);
            // Positive Z roll accelerates toward local-left, hence the minus sign.
            desiredRoll = (float) Mth.clamp(
                    -(desiredRightSpeed - currentRightSpeed) * HELI_SIDE_SPEED_TO_ROLL_GAIN,
                    -10.0D, 10.0D);
            horizontalPhase = distance <= 3.0D ? "FINAL_BRAKE" : "DIRECT_VELOCITY";
        } else {
            double yawError = Math.abs(Mth.wrapDegrees(desiredYaw - currentYaw));
            desiredForwardSpeed = Math.min(HELI_MAX_HORIZONTAL_SPEED,
                    Math.max(0.20D, distance * 0.035D));
            desiredPitch = yawError > 22.0D ? 0.0F : (float) Mth.clamp(
                    (desiredForwardSpeed - currentForwardSpeed) * HELI_FORWARD_SPEED_TO_PITCH_GAIN,
                    -6.0D, 14.0D);
            horizontalPhase = yawError > 22.0D ? "TURN_ALIGN" : "FORWARD_CRUISE";
        }
        if (facingTarget != null && vehicle.position().multiply(1.0D, 0.0D, 1.0D)
                .distanceTo(facingTarget.multiply(1.0D, 0.0D, 1.0D)) > 24.0D
                && hasFixedSelectedWeapon((AbstractVehicle) vehicle)) {
            Vec3 aimDelta = facingTarget.subtract(vehicle.position().add(0.0D, vehicle.getBbHeight() * 0.55D, 0.0D));
            double horizontalAimDistance = Math.max(1.0E-4D, aimDelta.horizontalDistance());
            float bodyAimPitch = (float) Mth.clamp(
                    -Math.toDegrees(Math.atan2(aimDelta.y, horizontalAimDistance)), -12.0D, 14.0D);
            desiredPitch = bodyAimPitch;
            horizontalPhase = "FIXED_WEAPON_AIM";
        }
        if (landing && distance >= 3.0D) desiredPitch = Math.min(desiredPitch, 4.0F);
        task.putBoolean(HELI_BRAKE_LATCH, false);

        double verticalVelocity = vehicle.getDeltaMovement().y;
        double verticalError = desiredY - vehicle.getY();
        double desiredVerticalSpeed = setCalculatedCollective((RotaryWingVehicle) vehicle, verticalError, landing, takingOff);
        boolean collectiveUp = false;
        boolean collectiveDown = false;
        boolean emergencyHover = reachedHorizontal && horizontalVelocity <= HELI_STATION_KEEPING_SPEED;
        RollControl rollControl = helicopterRollControl((RotaryWingVehicle) vehicle, desiredRoll);
        applyHelicopterControl(vehicle, desiredYaw, desiredPitch, rollControl.left(), rollControl.right(),
                false, false, emergencyHover);
        traceHelicopterControl(mob, vehicle, mode, target, navigationTarget, terrainY, desiredY, verticalError,
                desiredVerticalSpeed, distance, horizontalVelocity, desiredYaw, desiredPitch, desiredRoll,
                localForward, localRight, currentForwardSpeed, desiredForwardSpeed,
                currentRightSpeed, desiredRightSpeed, horizontalPhase,
                rollControl.left(), rollControl.right(), collectiveUp, collectiveDown, emergencyHover);
        if (reachedHorizontal && reachedVertical && horizontalVelocity < 0.035D && Math.abs(verticalVelocity) < 0.035D && !landing) {
            // Combat station-keeping must retain the target yaw. Using currentYaw
            // here cancelled the facing command on every settled attack tick.
            applyHelicopterControl(vehicle, facingTarget == null ? currentYaw : desiredYaw,
                    facingTarget == null ? 0.0F : desiredPitch,
                    false, false, false, false, false);
        }
        return true;
    }

    private static Vec3 helicopterCombatMovementTarget(Entity vehicle, Vec3 targetCenter, boolean visible) {
        Vec3 away = vehicle.position().subtract(targetCenter).multiply(1.0D, 0.0D, 1.0D);
        double distance = away.length();
        if (distance < 1.0E-4D) return vehicle.position();
        if (visible && distance <= ATTACK_RANGE) {
            return targetCenter.add(away.scale(STANDOFF_RANGE / distance));
        }
        return targetCenter;
    }

    private static void traceHelicopterControl(Mob pilot, Entity vehicle, String mode, Vec3 target, Vec3 navigationTarget,
                                               double terrainY, double desiredY, double verticalError,
                                               double desiredVerticalSpeed, double distance, double horizontalSpeed,
                                               float desiredYaw, float desiredPitch,
                                               float desiredRoll, double localForward, double localRight,
                                               double currentForwardSpeed, double desiredForwardSpeed,
                                               double currentRightSpeed, double desiredRightSpeed, String horizontalPhase,
                                               boolean rollLeft, boolean rollRight,
                                               boolean collectiveUp, boolean collectiveDown, boolean hoverBrake) {
        if (!YwzjVehicleCompatConfig.flightControlTraceEnabled() || vehicle == null || vehicle.level() == null) return;
        CompoundTag data = pilot.getPersistentData();
        long now = vehicle.level().getGameTime();
        if (data.getLong(HELI_LAST_CONTROL_TRACE_TICK) + YwzjVehicleCompatConfig.flightControlTraceIntervalTicks() > now) return;
        data.putLong(HELI_LAST_CONTROL_TRACE_TICK, now);
        Vec3 velocity = vehicle.getDeltaMovement();
        double currentRoll = vehicle instanceof AbstractVehicle ywzjVehicle ? ywzjVehicle.getZRot() : readFloat(invokeNoArg(vehicle, "getZRot"), 0.0F);
        double rollSpeed = vehicle instanceof RotaryWingVehicle helicopter ? helicopter.zRotSpeed : 0.0D;
        LOGGER.info("[DS-YWZJ-HELI] tick={} vehicle={} class={} name={} vehicleId={} mode={} hPhase={} pos={} target={} nav={} terrainY={} bottomY={} targetY={} altErr={} velocity={} vY={} desiredVY={} hSpeed={} distance={} yaw={} desiredYaw={} yawErr={} pitch={} desiredPitch={} roll={} desiredRoll={} rollSpeed={} localForward={} forwardSpeed={} desiredForwardSpeed={} localRight={} rightSpeed={} desiredRightSpeed={} power={} collectivePitch={} engineOn={} hover={} inputLeft={} inputRight={} inputUp={} inputDown={}",
                now, vehicle.getId(), vehicle.getClass().getName(), vehicle.getDisplayName().getString(),
                vehicle instanceof AbstractVehicle ywzjVehicle ? ywzjVehicle.getVehicleId() : "unknown",
                mode, horizontalPhase, fmt(vehicle.position()), fmt(target), fmt(navigationTarget), decimal(terrainY),
                decimal(helicopterBottomY(vehicle)), decimal(desiredY),
                decimal(verticalError), fmt(velocity), decimal(velocity.y), decimal(desiredVerticalSpeed),
                decimal(horizontalSpeed), decimal(distance),
                decimal(vehicle.getYRot()), decimal(desiredYaw), decimal(Mth.wrapDegrees(desiredYaw - vehicle.getYRot())),
                decimal(vehicle.getXRot()), decimal(desiredPitch), decimal(currentRoll), decimal(desiredRoll),
                decimal(rollSpeed), decimal(localForward), decimal(currentForwardSpeed), decimal(desiredForwardSpeed),
                decimal(localRight), decimal(currentRightSpeed), decimal(desiredRightSpeed),
                decimal(readFloat(invokeNoArg(vehicle, "getPower"), 0.0F)),
                decimal(readFloat(invokeNoArg(vehicle, "getCollectivePitch"), 0.0F)),
                vehicle instanceof AbstractVehicle ywzjVehicle && ywzjVehicle.isEngineOn(), hoverBrake,
                rollLeft, rollRight, collectiveUp, collectiveDown);
    }

    private static String decimal(double value) {
        return String.format(Locale.ROOT, "%.4f", value);
    }

    private static void applyHelicopterControl(Entity vehicle, float yaw, float pitch, boolean left, boolean right,
                                               boolean up, boolean down, boolean hoverBrake) {
        if (!(vehicle instanceof RotaryWingVehicle helicopter)) return;
        ControlUnit control = helicopter.controlUnit;
        control.forward = false;
        control.backward = false;
        control.left = left;
        control.right = right;
        control.up = up;
        control.down = down;
        control.leftYaw = false;
        control.rightYaw = false;
        control.xRot = pitch;
        control.yRot = yaw;
        control.xRotKeep = false;
        control.yRotKeep = false;
        helicopter.hoverMode = hoverBrake;
    }

    /**
     * Limitless exposes a target angle for pitch/yaw, but roll is rate controlled:
     * holding left/right keeps accelerating zRotSpeed. Treating the key as a target
     * caused the 9 -> 29 -> 49 -> 114 degree rollover visible in the flight trace.
     * This damped closed loop releases/reverses the key before the requested bank.
     */
    private static RollControl helicopterRollControl(RotaryWingVehicle helicopter, float desiredRoll) {
        double currentRoll = Mth.wrapDegrees(helicopter.getZRot());
        double rollError = Mth.wrapDegrees(desiredRoll - currentRoll);
        double rollSpeed = helicopter.zRotSpeed;
        double command = rollError - rollSpeed * 2.25D;
        if (Math.abs(currentRoll) > 18.0D) {
            command = -currentRoll - rollSpeed * 3.0D;
        }
        if (command > 0.55D) return new RollControl(false, true);
        if (command < -0.55D) return new RollControl(true, false);
        return new RollControl(false, false);
    }

    private record RollControl(boolean left, boolean right) {
    }

    private static void stopHelicopter(Entity vehicle) {
        applyHelicopterControl(vehicle, vehicle.getYRot(), 0.0F, false, false, false, false, false);
    }

    private static void hardStopHelicopter(Entity vehicle) {
        stopHelicopter(vehicle);
        if (vehicle instanceof RotaryWingVehicle helicopter) {
            helicopter.setCollectivePitch(0.0F);
            helicopter.setPower(0.0F);
            helicopter.setEngineSpeed(0.0F);
            helicopter.toggleEngine(Boolean.FALSE);
            helicopter.hoverMode = false;
            helicopter.xRotSpeed = 0.0F;
            helicopter.yRotSpeed = 0.0F;
            helicopter.zRotSpeed = 0.0F;
        }
        vehicle.setDeltaMovement(Vec3.ZERO);
    }

    /**
     * Finishes an explicit landing as one atomic transition. Limitless Vehicle
     * starts the engine when an entity enters seat zero, but does not stop it when
     * that entity leaves, so shutdown must happen both before and after dismount.
     * Clearing the completed task also allows the same pilot to board and take off
     * again later without being immediately treated as still landed.
     */
    private static void completeHelicopterLanding(Entity vehicle, Mob pilot) {
        CompoundTag task = pilot.getPersistentData();
        task.putString(HELI_MODE, "LANDED");
        hardStopHelicopter(vehicle);
        if (pilot.getVehicle() == vehicle) {
            com.arxyt.dominionsword.api.VehicleDismounts.dismount(vehicle, pilot);
        }
        hardStopHelicopter(vehicle);
        task.remove(HELI_MODE);
        task.remove(HELI_NAV_X);
        task.remove(HELI_NAV_Y);
        task.remove(HELI_NAV_Z);
        task.remove(HELI_HOLD_ALTITUDE);
        task.remove(HELI_LOCKED_ALTITUDE);
        task.remove(HELI_BRAKE_LATCH);
        task.remove(HELI_COMBAT_TARGET);
        unregisterActiveHelicopter(vehicle);
    }

    private static double setCalculatedCollective(RotaryWingVehicle helicopter, double altitudeError,
                                                  boolean landing, boolean takingOff) {
        double verticalSpeed = helicopter.getDeltaMovement().y;
        double desiredVerticalSpeed = landing
                ? Mth.clamp(altitudeError * 0.05D, -0.14D, 0.04D)
                : Mth.clamp(altitudeError * 0.045D, -0.18D, takingOff ? 0.16D : 0.18D);
        Object physics = readMember(helicopter, "physicsEngine");
        // PhysicsEngine.G is a positive downward acceleration (9.8 / 400).
        // Negating it silently reduced the controller baseline to the fallback 0.01.
        double gravity = Math.max(0.01D, Math.abs(readDouble(readMember(physics, "G"), 9.8D / 400.0D)));
        double rotorForce = Math.max(0.01D, readDouble(readMember(helicopter, "mainRotorForce"), 0.12D));
        double engineScale = Math.max(0.05D, helicopter.getPower() / 100.0D);
        double tilt = Math.max(0.72D, Math.cos(Math.toRadians(helicopter.getXRot()))
                * Math.cos(Math.toRadians(helicopter.getZRot())));
        double desiredAcceleration = gravity + Mth.clamp((desiredVerticalSpeed - verticalSpeed) * 0.18D, -0.035D, 0.035D);
        float collective = (float) Mth.clamp(100.0D * desiredAcceleration / (engineScale * rotorForce * tilt), 0.0D, 100.0D);
        float current = helicopter.getCollectivePitch();
        helicopter.setCollectivePitch((float) Mth.clamp(collective, current - 2.0F, current + 2.0F));
        return desiredVerticalSpeed;
    }

    private static void setHelicopterTask(Mob mob, Entity vehicle, String mode) {
        CompoundTag task = mob.getPersistentData();
        task.remove(HELI_COMBAT_TARGET);
        task.putString(HELI_MODE, mode);
        if ("TAKEOFF".equals(mode)) {
            task.remove(HELI_HOLD_ALTITUDE);
            task.remove(HELI_LOCKED_ALTITUDE);
        }
        task.putDouble(HELI_NAV_X, vehicle.getX());
        task.putDouble(HELI_NAV_Y, vehicle.getY());
        task.putDouble(HELI_NAV_Z, vehicle.getZ());
        if (mode.isBlank() || "LANDED".equals(mode)) unregisterActiveHelicopter(vehicle);
        else registerActiveHelicopter(vehicle);
    }

    private static void registerActiveHelicopter(Entity vehicle) {
        if (vehicle == null || vehicle.level().isClientSide()) return;
        ACTIVE_HELICOPTERS.computeIfAbsent(vehicle.level().dimension(), ignored -> ConcurrentHashMap.newKeySet()).add(vehicle.getUUID());
    }

    private static void unregisterActiveHelicopter(Entity vehicle) {
        if (vehicle == null) return;
        Set<UUID> entries = ACTIVE_HELICOPTERS.get(vehicle.level().dimension());
        if (entries != null) {
            entries.remove(vehicle.getUUID());
            if (entries.isEmpty()) ACTIVE_HELICOPTERS.remove(vehicle.level().dimension(), entries);
        }
    }

    private static boolean isPersistentGroundChassis(Entity vehicle) {
        return vehicle instanceof WheeledVehicle || isTrackedVehicle(vehicle);
    }

    private static void registerActiveGroundControl(Entity vehicle, boolean forward, boolean backward,
                                                    boolean right, boolean left, boolean brake) {
        if (!isPersistentGroundChassis(vehicle) || vehicle.level().isClientSide()) return;
        ACTIVE_GROUND_CONTROLS.computeIfAbsent(vehicle.level().dimension(), ignored -> new ConcurrentHashMap<>())
                .put(vehicle.getUUID(), new GroundControlState(forward, backward, right, left, brake));
    }

    private static void unregisterActiveGroundControl(Entity vehicle) {
        if (vehicle == null) return;
        Map<UUID, GroundControlState> entries = ACTIVE_GROUND_CONTROLS.get(vehicle.level().dimension());
        if (entries != null) {
            entries.remove(vehicle.getUUID());
            if (entries.isEmpty()) ACTIVE_GROUND_CONTROLS.remove(vehicle.level().dimension(), entries);
        }
        GROUND_CONTROL_TRACE_TICKS.remove(vehicle.getUUID());
    }

    private static Vec3 helicopterTaskTarget(Mob mob, Entity vehicle) {
        CompoundTag task = mob.getPersistentData();
        return new Vec3(task.contains(HELI_NAV_X) ? task.getDouble(HELI_NAV_X) : vehicle.getX(),
                task.contains(HELI_NAV_Y) ? task.getDouble(HELI_NAV_Y) : vehicle.getY(),
                task.contains(HELI_NAV_Z) ? task.getDouble(HELI_NAV_Z) : vehicle.getZ());
    }

    private static double helicopterCruiseAltitude(String mode) {
        return switch (mode) {
            case "HIGH_HOVER" -> HELI_HIGH_ALTITUDE;
            case "MEDIUM_HOVER" -> HELI_MEDIUM_ALTITUDE;
            default -> HELI_LOW_ALTITUDE;
        };
    }

    private static void ensureRoute(ServerPlayer player, Entity vehicle, Vec3 target, VehicleShape shape) {
        CompoundTag data = vehicle.getPersistentData();
        if (isTrackedVehicle(vehicle)) {
            ensureTrackedPoseRoute(player, vehicle, target, shape);
            return;
        }
        if (data.getBoolean(PATH_ASYNC_PENDING)) {
            advanceAsyncRoute(player, vehicle, target, Set.of(vehicle.getUUID()));
            return;
        }
        Vec3 safe = activeSafeTarget(vehicle, target);
        if (hasActiveRoute(vehicle, target) && data.contains(PATH_POINTS, Tag.TAG_LIST)) {
            if (data.getBoolean(PATH_BLOCKED)) {
                long now = vehicle.level().getGameTime();
                if (now < data.getLong(REPLAN_AFTER)) return;
                data.putLong(REPLAN_AFTER, now + PATH_REPLAN_COOLDOWN_TICKS);
                prepareRoute(player, vehicle, target, Set.of(vehicle.getUUID()));
                return;
            }
            if (pathPolicy(vehicle).equals(PATH_POLICY_DIRECT)) return;
            Vec3 nav = nextRoutePoint(vehicle, safe, shape, horizontalSpeed(vehicle));
            double check = nav == null ? 0.0D : Math.max(PATH_LOOKAHEAD_MIN, flatDistance(vehicle.position(), nav) + shape.radius());
            if (nav != null && canTravelDirect(vehicle, vehicle.position(), nav, shape, check)) return;
            long now = vehicle.level().getGameTime();
            if (now < data.getLong(REPLAN_AFTER)) return;
            data.putLong(REPLAN_AFTER, now + PATH_REPLAN_COOLDOWN_TICKS);
        }
        prepareRoute(player, vehicle, target, Set.of(vehicle.getUUID()));
    }

    private static List<FleetMove> fleetMoves(List<Entity> vehicles, List<Vec3> targets) {
        int count = Math.min(vehicles.size(), targets.size());
        List<FleetMove> moves = new ArrayList<>(count);
        for (int i = 0; i < count; i++) {
            Entity vehicle = vehicles.get(i);
            Vec3 target = targets.get(i);
            if (vehicle != null && target != null && isInstance(vehicle, VEHICLE_CLASS_NAME) && !isAirVehicle(vehicle)) moves.add(new FleetMove(i, vehicle, target));
        }
        return moves;
    }

    private List<Vec3> trackRoadFleetTargets(List<FleetMove> moves) {
        if (moves.size() <= 1) return moves.stream().map(FleetMove::target).toList();
        long now = moves.get(0).vehicle().level().getGameTime();
        cleanupTrackRoads(now);
        List<Vec3> result = new ArrayList<>(Collections.nCopies(moves.size(), null));
        FleetMove leader = moves.stream().min(Comparator.comparingDouble(FleetMove::distanceSqr)).orElse(moves.get(0));
        recordLeaderTrack(leader.vehicle(), now);
        int fleetRadius = fleetRadius(moves);
        Set<UUID> ignored = fleetVehicleIdSet(moves);
        Map<TrackRoadKey, Integer> roadUsers = new HashMap<>();
        for (FleetMove move : moves) {
            Vec3 target = move.target();
            if (move == leader) result.set(move.index(), target);
            else {
                Vec3 track = trackRoadTarget(move, fleetRadius, ignored, roadUsers, now);
                result.set(move.index(), track != null ? track : target);
            }
        }
        for (FleetMove move : moves) if (result.get(move.index()) == null) result.set(move.index(), move.target());
        return result;
    }

    private static void cleanupTrackRoads(long now) {
        TRACK_ROADS.entrySet().removeIf(entry -> entry.getValue().expiresAt() < now);
        TRACK_SAFETY_CACHE.entrySet().removeIf(entry -> entry.getValue().expiresAt() < now);
        LEADER_TRACKS.entrySet().removeIf(entry -> entry.getValue().isEmpty() || entry.getValue().peekLast().tick() + FLEET_TRACK_MAX_AGE_TICKS < now);
    }

    private static void recordLeaderTrack(Entity vehicle, long now) {
        Deque<TrackPoint> track = LEADER_TRACKS.computeIfAbsent(vehicle.getUUID(), id -> new ArrayDeque<>());
        Vec3 pos = vehicle.position();
        TrackPoint last = track.peekLast();
        if (last == null || flatDistance(last.position(), pos) >= 3.0D) {
            TrackPoint point = new TrackPoint(pos, now);
            track.addLast(point);
            while (track.size() > FLEET_TRACK_MAX_POINTS || (!track.isEmpty() && track.peekFirst().tick() + FLEET_TRACK_MAX_AGE_TICKS < now)) track.removeFirst();
            if (track.size() >= 3) {
                List<TrackPoint> points = new ArrayList<>(track);
                TrackRoadKey key = trackRoadKey(points, points.size() - 2);
                TRACK_ROADS.put(key, new TrackRoadSegment(points, now + FLEET_TRACK_MAX_AGE_TICKS));
            }
        }
    }

    private Vec3 trackRoadTarget(FleetMove follower, int fleetRadius, Set<UUID> ignored, Map<TrackRoadKey, Integer> roadUsers, long now) {
        Entity vehicle = follower.vehicle();
        Vec3 position = vehicle.position();
        Vec3 toFinal = follower.target().subtract(position).multiply(1.0D, 0.0D, 1.0D);
        if (toFinal.lengthSqr() < 4.0D) return null;
        toFinal = toFinal.normalize();
        VehicleShape shape = VehicleShape.from(vehicle);
        double footprint = Math.max(2.0D, shape.radius() * 2.0D);
        double accessRadius = Math.max(FLEET_TRACK_ROAD_ACCESS_BASE, footprint * 1.8D);
        double accessRadiusSqr = accessRadius * accessRadius;
        double lookahead = Math.max(FLEET_TRACK_ROAD_LOOKAHEAD_BASE, footprint * 1.35D);
        TrackRoadMatch best = null;
        double bestScore = Double.NEGATIVE_INFINITY;
        int scanned = 0;
        for (Map.Entry<TrackRoadKey, TrackRoadSegment> entry : TRACK_ROADS.entrySet()) {
            TrackRoadSegment segment = entry.getValue();
            if (segment == null || segment.expiresAt() < now || segment.points().size() < 3) continue;
            if (++scanned > TRACK_ROAD_SCAN_LIMIT) break;
            List<TrackPoint> points = segment.points();
            int closest = -1;
            double closestDistanceSqr = Double.MAX_VALUE;
            for (int i = 0; i < points.size(); i++) {
                TrackPoint point = points.get(i);
                if (point.tick() + FLEET_TRACK_MAX_AGE_TICKS < now) continue;
                Vec3 p = point.position();
                if (Math.abs(p.x - position.x) > accessRadius || Math.abs(p.z - position.z) > accessRadius) continue;
                double distanceSqr = flatDistanceSqr(position, p);
                if (distanceSqr < closestDistanceSqr) {
                    closestDistanceSqr = distanceSqr;
                    closest = i;
                }
            }
            if (closest < 0 || closestDistanceSqr > accessRadiusSqr) continue;
            TrackRoadAdvance advance = advanceOnTrackIndexed(points, closest, lookahead, now);
            if (advance == null) continue;
            Vec3 candidate = advance.position();
            Vec3 toCandidate = candidate.subtract(position).multiply(1.0D, 0.0D, 1.0D);
            if (toCandidate.lengthSqr() < 9.0D) continue;
            double alignment = dot2d(toCandidate.normalize(), toFinal);
            if (alignment < 0.12D) continue;
            double progress = flatDistance(position, follower.target()) - flatDistance(candidate, follower.target());
            if (progress < -footprint * 0.35D) continue;
            double score = progress * 1.8D - Math.sqrt(closestDistanceSqr) * 0.8D + alignment * 8.0D;
            if (score > bestScore) {
                bestScore = score;
                best = new TrackRoadMatch(points, closest, entry.getKey());
            }
        }
        if (best == null) return null;
        int users = roadUsers.merge(best.key(), 1, Integer::sum);
        double spacing = Math.max(6.0D, footprint + 2.0D);
        return safeStringPulledTrackTarget(follower, best.points(), best.closestIndex(), lookahead + (users - 1) * spacing, now, fleetRadius, ignored);
    }

    private Vec3 safeStringPulledTrackTarget(FleetMove follower, List<TrackPoint> points, int startIndex, double lookahead, long now, int fleetRadius, Set<UUID> ignored) {
        TrackRoadAdvance base = advanceOnTrackIndexed(points, startIndex, lookahead, now);
        if (base == null) return null;
        int last = Math.min(points.size() - 1, base.index() + 4);
        Vec3 fallback = base.position();
        int safetyChecks = 0;
        for (int i = last; i >= base.index() && safetyChecks < 3; i--) {
            TrackPoint point = points.get(i);
            if (point.tick() + FLEET_TRACK_MAX_AGE_TICKS < now) continue;
            Vec3 candidate = point.position();
            if (!isUsefulTrackRoadTarget(follower, candidate)) continue;
            safetyChecks++;
            if (isSafeTrackTarget(follower, candidate, fleetRadius, ignored)) return candidate;
        }
        return isSafeTrackTarget(follower, fallback, fleetRadius, ignored) ? fallback : null;
    }

    private boolean isSafeTrackTarget(FleetMove follower, Vec3 target, int fleetRadius, Set<UUID> ignored) {
        if (target == null) return false;
        long now = follower.vehicle().level().getGameTime();
        TrackSafetyKey key = new TrackSafetyKey(follower.vehicle().getUUID(), quantizeTrack(target.x), quantizeTrack(target.z), fleetRadius);
        CachedTrackSafety cached = TRACK_SAFETY_CACHE.get(key);
        if (cached != null && cached.expiresAt() >= now) return cached.safe();
        boolean safe = canTravelDirect(follower.vehicle(), follower.vehicle().position(), target, VehicleShape.from(follower.vehicle()), flatDistance(follower.vehicle().position(), target) + fleetRadius);
        TRACK_SAFETY_CACHE.put(key, new CachedTrackSafety(safe, now + TRACK_SAFETY_CACHE_TICKS));
        return safe;
    }

    private static boolean isUsefulTrackRoadTarget(FleetMove follower, Vec3 target) {
        if (target == null) return false;
        Vec3 pos = follower.vehicle().position();
        Vec3 toFinal = follower.target().subtract(pos).multiply(1.0D, 0.0D, 1.0D);
        Vec3 toTrack = target.subtract(pos).multiply(1.0D, 0.0D, 1.0D);
        if (toFinal.lengthSqr() < 1.0E-6D || toTrack.lengthSqr() < 1.0E-6D) return false;
        if (dot2d(toFinal.normalize(), toTrack.normalize()) < 0.05D) return false;
        return flatDistance(target, follower.target()) < flatDistance(pos, follower.target()) + 4.0D;
    }

    private static TrackRoadAdvance advanceOnTrackIndexed(List<TrackPoint> points, int startIndex, double lookahead, long now) {
        Vec3 previous = points.get(startIndex).position();
        double walked = 0.0D;
        for (int i = startIndex + 1; i < points.size(); i++) {
            TrackPoint point = points.get(i);
            if (point.tick() + FLEET_TRACK_MAX_AGE_TICKS < now) continue;
            Vec3 current = point.position();
            walked += flatDistance(previous, current);
            if (walked >= lookahead) return new TrackRoadAdvance(current, i);
            previous = current;
        }
        TrackPoint last = points.get(points.size() - 1);
        return last.tick() + FLEET_TRACK_MAX_AGE_TICKS < now ? null : new TrackRoadAdvance(last.position(), points.size() - 1);
    }

    private static TrackRoadKey trackRoadKey(List<TrackPoint> points, int index) {
        Vec3 point = points.get(index).position();
        Vec3 before = points.get(Math.max(0, index - 1)).position();
        Vec3 after = points.get(Math.min(points.size() - 1, index + 1)).position();
        Vec3 dir = after.subtract(before).multiply(1.0D, 0.0D, 1.0D);
        if (dir.lengthSqr() < 1.0E-6D) dir = new Vec3(0.0D, 0.0D, 1.0D);
        dir = dir.normalize();
        return new TrackRoadKey(Mth.floor(point.x / 6.0D), Mth.floor(point.z / 6.0D), Mth.floor((dir.x + 1.0D) * 4.0D), Mth.floor((dir.z + 1.0D) * 4.0D));
    }

    private static int quantizeTrack(double value) {
        return Mth.floor(value * 0.5D);
    }

    private static int fleetRadius(List<FleetMove> moves) {
        int radius = 1;
        for (FleetMove move : moves) radius = Math.max(radius, (int) Math.ceil(VehicleShape.from(move.vehicle()).radius()));
        return radius;
    }

    private static Set<UUID> fleetVehicleIdSet(List<FleetMove> moves) {
        Set<UUID> ids = new HashSet<>();
        for (FleetMove move : moves) ids.add(move.vehicle().getUUID());
        return ids;
    }

    private static double dot2d(Vec3 a, Vec3 b) {
        return a.x * b.x + a.z * b.z;
    }

    private static double flatDistanceSqr(Vec3 a, Vec3 b) {
        double dx = a.x - b.x;
        double dz = a.z - b.z;
        return dx * dx + dz * dz;
    }

    private static List<Vec3> assignedFleetTargets(List<Entity> vehicles, List<Vec3> targets) {
        int count = Math.min(vehicles.size(), targets.size());
        if (count <= 1) return targets;
        List<Vec3> result = new ArrayList<>(Collections.nCopies(targets.size(), null));
        boolean[] used = new boolean[count];
        Vec3 vehicleCenter = centerOfVehicles(vehicles, count);
        Vec3 targetCenter = centerOfTargets(targets, count);
        Vec3 axis = longestTargetAxis(targets, count);
        Vec3 perpendicular = new Vec3(-axis.z, 0.0D, axis.x);
        List<Integer> vehicleOrder = new ArrayList<>();
        List<Integer> targetOrder = new ArrayList<>();
        for (int i = 0; i < count; i++) {
            vehicleOrder.add(i);
            targetOrder.add(i);
        }
        vehicleOrder.sort(Comparator.comparingDouble(i -> vehicles.get(i).position().subtract(vehicleCenter).dot(perpendicular)));
        targetOrder.sort(Comparator.comparingDouble(i -> targets.get(i).subtract(targetCenter).dot(perpendicular)));
        for (int order = 0; order < count; order++) {
            int vehicleIndex = vehicleOrder.get(order);
            int preferred = targetOrder.get(order);
            int best = preferred;
            double bestScore = scoreFleetAssignment(vehicles.get(vehicleIndex), targets.get(preferred), order, order);
            for (int targetIndex = 0; targetIndex < count; targetIndex++) {
                if (used[targetIndex]) continue;
                int targetRank = targetOrder.indexOf(targetIndex);
                double score = scoreFleetAssignment(vehicles.get(vehicleIndex), targets.get(targetIndex), order, targetRank);
                if (score < bestScore || used[best]) {
                    bestScore = score;
                    best = targetIndex;
                }
            }
            used[best] = true;
            result.set(vehicleIndex, targets.get(best));
        }
        for (int i = 0; i < targets.size(); i++) if (result.get(i) == null) result.set(i, targets.get(i));
        return result;
    }

    private static double scoreFleetAssignment(Entity vehicle, Vec3 target, int vehicleRank, int targetRank) {
        return flatDistance(vehicle.position(), target) + Math.abs(vehicleRank - targetRank) * 12.0D;
    }

    private static Vec3 centerOfVehicles(List<Entity> vehicles, int count) {
        double x = 0.0D, z = 0.0D;
        for (int i = 0; i < count; i++) {
            x += vehicles.get(i).getX();
            z += vehicles.get(i).getZ();
        }
        return new Vec3(x / count, 0.0D, z / count);
    }

    private static Vec3 centerOfTargets(List<Vec3> targets, int count) {
        double x = 0.0D, z = 0.0D;
        for (int i = 0; i < count; i++) {
            x += targets.get(i).x;
            z += targets.get(i).z;
        }
        return new Vec3(x / count, 0.0D, z / count);
    }

    private static Vec3 longestTargetAxis(List<Vec3> targets, int count) {
        Vec3 best = new Vec3(1.0D, 0.0D, 0.0D);
        double bestDistance = 0.0D;
        for (int i = 0; i < count; i++) for (int j = i + 1; j < count; j++) {
            Vec3 delta = targets.get(j).subtract(targets.get(i)).multiply(1.0D, 0.0D, 1.0D);
            double distance = delta.lengthSqr();
            if (distance > bestDistance && distance > 1.0E-6D) {
                bestDistance = distance;
                best = delta.normalize();
            }
        }
        return best;
    }

    private static void prepareRoute(ServerPlayer player, Entity vehicle, Vec3 target, Set<UUID> ignoredVehicles) {
        try (LagTrace ignoredTrace = LagTrace.start("ywzj.route.prepare", "vehicle=" + (vehicle == null ? "null" : vehicle.getId()))) {
        if (vehicle == null || target == null || !isInstance(vehicle, VEHICLE_CLASS_NAME)) return;
        if (isTrackedVehicle(vehicle)) {
            prepareTrackedPoseRoute(player, vehicle, target, ignoredVehicles);
            return;
        }
        CompoundTag data = vehicle.getPersistentData();
        if (data.getBoolean(PATH_ASYNC_PENDING)) {
            advanceAsyncRoute(player, vehicle, target, ignoredVehicles);
            return;
        }
        if (hasActiveRoute(vehicle, target)
                && vehicle.getPersistentData().contains(PATH_POINTS, Tag.TAG_LIST)
                && !vehicle.getPersistentData().getBoolean(PATH_BLOCKED)) return;
        VehicleShape shape = VehicleShape.from(vehicle);
        traceRouteShape(vehicle, shape);
        LagTrace.mark("shape");
        Vec3 safe = safeTargetNear(vehicle, target, shape, ignoredVehicles);
        LagTrace.mark("safe_target");
        long generation = data.getLong(PATH_GENERATION) + 1L;
        data.putLong(PATH_GENERATION, generation);
        data.putDouble(FINAL_TARGET_X, target.x);
        data.putDouble(FINAL_TARGET_Z, target.z);
        data.putDouble(SAFE_TARGET_X, safe.x);
        data.putDouble(SAFE_TARGET_Y, safe.y);
        data.putDouble(SAFE_TARGET_Z, safe.z);
        data.remove(PATH_BLOCKED);
        List<Vec3> route;
        double directDistance = Math.max(PATH_LOOKAHEAD_MIN, flatDistance(vehicle.position(), safe) + shape.radius());
        boolean direct = canTravelDirect(vehicle, vehicle.position(), safe, shape, directDistance);
        LagTrace.mark("direct_check:" + direct);
        if (pathPolicy(vehicle).equals(PATH_POLICY_DIRECT) || (!pathPolicy(vehicle).equals(PATH_POLICY_ALWAYS) && direct)) {
            route = List.of(vehicle.position(), safe);
        } else {
            if (startAsyncRoute(vehicle, safe, shape, ignoredVehicles, generation)) {
                data.putBoolean(PATH_ASYNC_PENDING, true);
                data.putLong(REPLAN_AFTER, vehicle.level().getGameTime() + PATH_REPLAN_COOLDOWN_TICKS);
                storeRoute(vehicle, safe, List.of(vehicle.position(), vehicle.position()), 1);
                return;
            }
            route = findAvoidancePath(vehicle, vehicle.position(), safe, shape, ignoredVehicles);
            LagTrace.mark("astar:size=" + route.size());
            if (route.size() <= 1) {
                data.putLong(REPLAN_AFTER, vehicle.level().getGameTime() + PATH_REPLAN_COOLDOWN_TICKS);
                route = List.of(vehicle.position(), vehicle.position());
                pathDebug(vehicle, "ROUTE_BLOCKED", "target=%s safe=%s direct=%s reason=no_avoidance_path", fmt(target), fmt(safe), direct);
            }
            else {
                route = simplifyRoute(vehicle, route, shape);
                LagTrace.mark("simplify:size=" + route.size());
                if (flatDistance(route.get(route.size() - 1), safe) > 1.0D) {
                    List<Vec3> appended = new ArrayList<>(route);
                    appended.add(safe);
                    route = appended;
                }
            }
        }
        storeRoute(vehicle, safe, route, firstUsefulIndex(route, vehicle.position(), shape));
        refreshPlannedPath(player, vehicle, route);
        LagTrace.mark("store_route:points=" + route.size());
        pathDebug(vehicle, "ROUTE_PREPARED", "target=%s safe=%s direct=%s points=%d policy=%s", fmt(target), fmt(safe), direct, route.size(), pathPolicy(vehicle));
        }
    }

    /**
     * Tracked vehicles are planned in configuration space: position plus hull heading.
     * The former grid planner only knew the centre position, so it could select a gap that
     * was reachable on paper but could not be entered, turned through, or exited by the tank.
     */
    private static void ensureTrackedPoseRoute(ServerPlayer player, Entity vehicle, Vec3 target, VehicleShape shape) {
        if (TRACKED_RECOVERIES.containsKey(vehicle.getUUID())) return;
        TrackedPoseRoute route = TRACKED_POSE_ROUTES.get(vehicle.getUUID());
        if (route != null && route.matches(target)) return;
        TrackedPoseRouteBuild build = TRACKED_POSE_BUILDS.get(vehicle.getUUID());
        if (build != null && build.matches(target)) {
            build.commandTarget = target;
            advanceTrackedPoseRoute(player, vehicle, build);
            return;
        }
        // A failed search must remain failed until its cooldown expires.  Previously the
        // missing route caused every command callback to create another fresh 3,600-node
        // search, so a blocked tank did expensive work repeatedly without ever progressing.
        if (trackedPoseReplanCoolingDown(vehicle, target)) return;
        prepareTrackedPoseRoute(player, vehicle, target, Set.of(vehicle.getUUID()));
    }

    private static void prepareTrackedPoseRoute(ServerPlayer player, Entity vehicle, Vec3 target, Set<UUID> ignoredVehicles) {
        if (!(vehicle instanceof AbstractVehicle chassis) || target == null) return;
        if (TRACKED_RECOVERIES.containsKey(vehicle.getUUID())) return;
        TrackedPoseRoute active = TRACKED_POSE_ROUTES.get(vehicle.getUUID());
        if (active != null && active.matches(target)) return;
        TrackedPoseRouteBuild existing = TRACKED_POSE_BUILDS.get(vehicle.getUUID());
        if (existing != null && existing.matches(target)) {
            existing.commandTarget = target;
            advanceTrackedPoseRoute(player, vehicle, existing);
            return;
        }
        // prepareFleetMoveRoutes() may call this method every server tick.  Respect the same
        // failure cooldown here as in ensureTrackedPoseRoute(), otherwise an unsolved route
        // is destroyed and rebuilt before the incremental search can make a second step.
        if (trackedPoseReplanCoolingDown(vehicle, target)) return;

        CompoundTag data = vehicle.getPersistentData();
        ASYNC_ROUTES.remove(vehicle.getUUID());
        data.remove(PATH_ASYNC_PENDING);
        TRACKED_POSE_ROUTES.remove(vehicle.getUUID());
        TRACKED_POSE_BUILDS.remove(vehicle.getUUID());

        VehicleShape shape = VehicleShape.from(vehicle);
        TrackedHull hull = TrackedHull.from(chassis);
        if (hull == null) {
            data.putBoolean(PATH_BLOCKED, true);
            data.putLong(REPLAN_AFTER, vehicle.level().getGameTime() + 40L);
            pathDebug(vehicle, "POSE_ROUTE_BLOCKED", "reason=missing_main_obb target=%s", fmt(target));
            return;
        }
        pathDebug(vehicle, "POSE_HULL", "groundOffset=%.3f lowSideY=%.3f samples=%d entityY=%.3f mainObb=%s",
                hull.entityGroundOffset, hull.lowSideContactY, hull.samples.size(), vehicle.getY(), boxBounds(mainObbAabb(vehicle)));
        pathDebug(vehicle, "POSE_NAV_FRAME", "mode=2.5d_yaw_only physicalPitch=%.2f physicalRoll=%.2f",
                vehicle.getXRot(), chassis.getZRot());
        long generation = data.getLong(PATH_GENERATION) + 1L;
        data.putLong(PATH_GENERATION, generation);
        data.putDouble(FINAL_TARGET_X, target.x);
        data.putDouble(FINAL_TARGET_Z, target.z);
        data.putDouble(SAFE_TARGET_X, target.x);
        data.putDouble(SAFE_TARGET_Y, target.y);
        data.putDouble(SAFE_TARGET_Z, target.z);
        data.remove(PATH_BLOCKED);

        List<TrackedPose> direct = directTrackedPoseRoute(vehicle, target, hull, ignoredVehicles);
        if (direct != null) {
            installTrackedPoseRoute(player, vehicle, target, direct.get(direct.size() - 1).position(), generation, hull, direct);
            pathDebug(vehicle, "POSE_ROUTE_DIRECT", "target=%s steps=%d", fmt(target), direct.size());
            return;
        }

        double distance = flatDistance(vehicle.position(), target);
        double range = Mth.clamp(Math.max(28.0D, distance + shape.radius() * 4.0D), 28.0D, PATH_SEARCH_RADIUS);
        TrackedPoseRouteBuild build = new TrackedPoseRouteBuild(vehicle.position(), vehicle.getYRot(), target, hull,
                ignoredVehicles == null ? Set.of() : Set.copyOf(ignoredVehicles), generation, range,
                vehicle.level().getGameTime());
        TRACKED_POSE_BUILDS.put(vehicle.getUUID(), build);
        data.putLong(REPLAN_AFTER, vehicle.level().getGameTime() + PATH_REPLAN_COOLDOWN_TICKS);
        pathDebug(vehicle, "POSE_ROUTE_START", "target=%s range=%.1f headings=%d", fmt(target), range, TRACKED_POSE_HEADINGS);
        advanceTrackedPoseRoute(player, vehicle, build);
    }

    private static boolean trackedPoseReplanCoolingDown(Entity vehicle, Vec3 target) {
        if (vehicle == null || target == null) return false;
        CompoundTag data = vehicle.getPersistentData();
        if (!data.getBoolean(PATH_BLOCKED) || !data.contains(FINAL_TARGET_X) || !data.contains(FINAL_TARGET_Z)) return false;
        Vec3 failedTarget = new Vec3(data.getDouble(FINAL_TARGET_X), target.y, data.getDouble(FINAL_TARGET_Z));
        return flatDistanceSqr(target, failedTarget) <= 4.0D
                && vehicle.level().getGameTime() < data.getLong(REPLAN_AFTER);
    }

    private static List<TrackedPose> directTrackedPoseRoute(Entity vehicle, Vec3 target, TrackedHull hull, Set<UUID> ignoredVehicles) {
        Vec3 start = vehicle.position();
        Vec3 horizontal = target.subtract(start).multiply(1.0D, 0.0D, 1.0D);
        if (horizontal.lengthSqr() < 1.0E-6D) return List.of(new TrackedPose(start, vehicle.getYRot(), false));
        float yaw = yawTo(horizontal);
        if (trackedShouldDirectShortReverse(vehicle.getYRot(), yaw, horizontal.length())) {
            float reverseYaw = Mth.wrapDegrees(yaw + 180.0F);
            List<TrackedPose> route = new ArrayList<>();
            route.add(new TrackedPose(start, vehicle.getYRot(), true));
            int segments = Math.max(1, Mth.ceil(horizontal.length() / TRACKED_POSE_FORWARD_STEP));
            Vec3 previous = start;
            for (int i = 1; i <= segments; i++) {
                double fraction = (double) i / segments;
                Vec3 raw = new Vec3(Mth.lerp(fraction, start.x, target.x), previous.y,
                        Mth.lerp(fraction, start.z, target.z));
                Vec3 next = resolveTrackedPosePosition(vehicle, raw, reverseYaw, hull, ignoredVehicles, previous.y);
                if (next == null || !terrainStepAllowed(previous.y, next.y)) return null;
                route.add(new TrackedPose(next, reverseYaw, true));
                previous = next;
            }
            pathDebug(vehicle, "POSE_ROUTE_SHORT_REVERSE", "target=%s distance=%.2f bodyYaw=%.1f reverseYaw=%.1f steps=%d",
                    fmt(target), horizontal.length(), vehicle.getYRot(), reverseYaw, route.size());
            return route;
        }
        if (!canTrackedPosePivot(vehicle, start, vehicle.getYRot(), yaw, hull, ignoredVehicles)) return null;
        List<TrackedPose> route = new ArrayList<>();
        route.add(new TrackedPose(start, vehicle.getYRot(), false));
        if (Math.abs(Mth.wrapDegrees(yaw - vehicle.getYRot())) > TRACKED_POSE_YAW_TOLERANCE) route.add(new TrackedPose(start, yaw, false));
        // Do not represent a 40-block clear road as a single control target.  The native
        // tracked chassis can cover half a block per physics tick; command callbacks are much
        // sparser, so a single endpoint guaranteed that it would overshoot the destination.
        int segments = Math.max(1, Mth.ceil(horizontal.length() / TRACKED_POSE_FORWARD_STEP));
        Vec3 previous = start;
        for (int i = 1; i <= segments; i++) {
            double fraction = (double) i / segments;
            Vec3 raw = new Vec3(Mth.lerp(fraction, start.x, target.x), previous.y, Mth.lerp(fraction, start.z, target.z));
            Vec3 next = resolveTrackedPosePosition(vehicle, raw, yaw, hull, ignoredVehicles, previous.y);
            if (next == null || !terrainStepAllowed(previous.y, next.y)) return null;
            route.add(new TrackedPose(next, yaw, false));
            previous = next;
        }
        return route;
    }

    static boolean trackedShouldDirectShortReverse(float currentYaw, float targetYaw, double distance) {
        return distance < TRACKED_POSE_DIRECT_REVERSE_RANGE
                && Math.abs(Mth.wrapDegrees(targetYaw - currentYaw)) >= REVERSE_ANGLE;
    }

    private static void advanceTrackedPoseRoute(ServerPlayer player, Entity vehicle, TrackedPoseRouteBuild build) {
        CompoundTag data = vehicle.getPersistentData();
        if (!build.matchesFinalTarget(data) || !(vehicle instanceof AbstractVehicle)) {
            TRACKED_POSE_BUILDS.remove(vehicle.getUUID());
            return;
        }
        long gameTick = vehicle.level().getGameTime();
        // prepareRoute(), the command callback and tickGroundControl() may all reach here in
        // one server tick.  A per-call budget would silently multiply under exactly the load
        // that pathfinding is supposed to survive.
        if (build.lastAdvanceTick == gameTick) return;
        build.lastAdvanceTick = gameTick;
        long sliceStart = System.nanoTime();
        long deadline = sliceStart + TRACKED_POSE_SEARCH_BUDGET_NANOS;
        int budget = TRACKED_POSE_EXPANSIONS_PER_TICK;
        int expandedThisTick = 0;
        while (budget-- > 0 && (expandedThisTick == 0 || System.nanoTime() < deadline)) {
            TrackedPoseNode node = build.open.poll();
            if (node == null) {
                build.cpuNanos += System.nanoTime() - sliceStart;
                failTrackedPoseRoute(vehicle, build, "exhausted");
                return;
            }
            Double best = build.costs.get(node.key);
            if (best == null || node.cost > best + 1.0E-6D) continue;
            pathDebug(vehicle, "POSE_EXPAND", "generation=%d expanded=%d key=(%d,%d,%d) node=%s yaw=%.1f g=%.3f f=%.3f open=%d",
                    build.generation, build.expanded, node.key.x, node.key.z, node.key.heading,
                    fmt(node.position), trackedPoseYaw(node.key.heading), node.cost, node.priority, build.open.size());
            if (flatDistance(node.position, build.target) <= TRACKED_POSE_GOAL_RADIUS) {
                List<TrackedPose> rawRoute = reconstructTrackedPoseRoute(node, build.startYaw);
                List<TrackedPose> route = simplifyTrackedPoseRoute(vehicle, rawRoute, build.hull, build.ignored);
                build.cpuNanos += System.nanoTime() - sliceStart;
                TRACKED_POSE_BUILDS.remove(vehicle.getUUID());
                installTrackedPoseRoute(player, vehicle, build.commandTarget, node.position, build.generation, build.hull, route);
                pathDebug(vehicle, "POSE_ROUTE_APPLIED", "generation=%d ticks=%d cpuMs=%.2f expanded=%d poseTests=%d cacheHits=%d obbFallbacks=%d terrainContacts=%d convexRejects=%d rejected=%d sweeps=%d rawSteps=%d simplifiedSteps=%d final=%s",
                        build.generation, gameTick - build.startedTick + 1L, build.cpuNanos / 1_000_000.0D,
                        build.expanded, build.poseTests, build.cacheHits, build.obbFallbacks,
                        build.terrainContacts, build.convexRejects, build.rejectedPoses, build.sweeps,
                        rawRoute.size(), route.size(), fmt(node.position));
                return;
            }
            if (++build.expanded > TRACKED_POSE_MAX_EXPANSIONS) {
                build.cpuNanos += System.nanoTime() - sliceStart;
                failTrackedPoseRoute(vehicle, build, "budget");
                return;
            }
            expandedThisTick++;
            expandTrackedPoseNode(vehicle, build, node);
        }
        build.cpuNanos += System.nanoTime() - sliceStart;
        if (gameTick - build.startedTick + 1L >= TRACKED_POSE_PARTIAL_ROUTE_TICKS
                && build.bestNode != null
                && build.bestNode.parent != null
                && flatDistance(build.anchor, build.bestNode.position) >= TRACKED_POSE_PARTIAL_MIN_PROGRESS) {
            List<TrackedPose> rawRoute = reconstructTrackedPoseRoute(build.bestNode, build.startYaw);
            List<TrackedPose> route = simplifyTrackedPoseRoute(vehicle, rawRoute, build.hull, build.ignored);
            TRACKED_POSE_BUILDS.remove(vehicle.getUUID());
            installTrackedPoseRoute(player, vehicle, build.commandTarget, build.bestNode.position,
                    build.generation, build.hull, route);
            pathDebug(vehicle, "POSE_ROUTE_PARTIAL", "generation=%d ticks=%d cpuMs=%.2f expanded=%d rawSteps=%d simplifiedSteps=%d progress=%.2f remaining=%.2f",
                    build.generation, gameTick - build.startedTick + 1L, build.cpuNanos / 1_000_000.0D,
                    build.expanded, rawRoute.size(), route.size(), flatDistance(build.anchor, build.bestNode.position),
                    flatDistance(build.bestNode.position, build.target));
            return;
        }
        if (gameTick - build.lastProgressTick >= 20L) {
            build.lastProgressTick = gameTick;
            pathDebug(vehicle, "POSE_ROUTE_PROGRESS", "generation=%d ticks=%d sliceMs=%.2f cpuMs=%.2f tickExpanded=%d expanded=%d open=%d poseTests=%d cacheHits=%d obbFallbacks=%d terrainContacts=%d convexRejects=%d rejected=%d sweeps=%d",
                    build.generation, gameTick - build.startedTick + 1L,
                    (System.nanoTime() - sliceStart) / 1_000_000.0D, build.cpuNanos / 1_000_000.0D,
                    expandedThisTick, build.expanded, build.open.size(), build.poseTests,
                    build.cacheHits, build.obbFallbacks, build.terrainContacts, build.convexRejects,
                    build.rejectedPoses, build.sweeps);
        }
    }

    private static void expandTrackedPoseNode(Entity vehicle, TrackedPoseRouteBuild build, TrackedPoseNode node) {
        int left = Math.floorMod(node.key.heading - 1, TRACKED_POSE_HEADINGS);
        int right = Math.floorMod(node.key.heading + 1, TRACKED_POSE_HEADINGS);
        addTrackedPoseRotation(vehicle, build, node, left);
        addTrackedPoseRotation(vehicle, build, node, right);
        addTrackedPoseTranslation(vehicle, build, node, false);
        addTrackedPoseTranslation(vehicle, build, node, true);
    }

    private static void addTrackedPoseRotation(Entity vehicle, TrackedPoseRouteBuild build, TrackedPoseNode from, int heading) {
        float yaw = trackedPoseYaw(heading);
        if (!canTrackedPoseOccupy(vehicle, from.position, yaw, build.hull, build.ignored)) {
            pathDebug(vehicle, "POSE_CANDIDATE_REJECT", "action=rotate from=(%d,%d,%d) toHeading=%d yaw=%.1f reason=occupancy",
                    from.key.x, from.key.z, from.key.heading, heading, yaw);
            return;
        }
        TrackedPoseKey key = new TrackedPoseKey(from.key.x, from.key.z, heading);
        pathDebug(vehicle, "POSE_CANDIDATE_PASS", "action=rotate from=(%d,%d,%d) to=(%d,%d,%d) pos=%s yaw=%.1f",
                from.key.x, from.key.z, from.key.heading, key.x, key.z, key.heading, fmt(from.position), yaw);
        addTrackedPoseNode(vehicle, build, key, from.position, from, TRACKED_POSE_ROTATION_COST, false);
    }

    private static void addTrackedPoseTranslation(Entity vehicle, TrackedPoseRouteBuild build, TrackedPoseNode from, boolean reverse) {
        float yaw = trackedPoseYaw(from.key.heading);
        Vec3 direction = Vec3.directionFromRotation(0.0F, yaw).multiply(1.0D, 0.0D, 1.0D).normalize();
        if (reverse) direction = direction.scale(-1.0D);
        Vec3 raw = from.position.add(direction.scale(TRACKED_POSE_FORWARD_STEP));
        TrackedPoseKey key = trackedPoseKey(build.anchor, raw, from.key.heading);
        Vec3 quantized = trackedPosePosition(build.anchor, key, from.position.y);
        if (flatDistance(quantized, from.position) < 0.25D) {
            pathDebug(vehicle, "POSE_CANDIDATE_REJECT", "action=%s from=(%d,%d,%d) raw=%s quantized=%s reason=short_edge",
                    reverse ? "reverse" : "forward", from.key.x, from.key.z, from.key.heading, fmt(raw), fmt(quantized));
            return;
        }
        if (!build.inRange(quantized)) {
            pathDebug(vehicle, "POSE_CANDIDATE_REJECT", "action=%s from=(%d,%d,%d) target=%s reason=range",
                    reverse ? "reverse" : "forward", from.key.x, from.key.z, from.key.heading, fmt(quantized));
            return;
        }
        Vec3 end = sweepTrackedPose(vehicle, from.position, quantized, yaw, build.hull, build.ignored, reverse);
        if (end == null) {
            pathDebug(vehicle, "POSE_CANDIDATE_REJECT", "action=%s from=(%d,%d,%d) target=%s yaw=%.1f reason=sweep",
                    reverse ? "reverse" : "forward", from.key.x, from.key.z, from.key.heading, fmt(quantized), yaw);
            return;
        }
        pathDebug(vehicle, "POSE_CANDIDATE_PASS", "action=%s from=(%d,%d,%d) to=(%d,%d,%d) planned=%s resolved=%s yaw=%.1f",
                reverse ? "reverse" : "forward", from.key.x, from.key.z, from.key.heading,
                key.x, key.z, key.heading, fmt(quantized), fmt(end), yaw);
        addTrackedPoseNode(vehicle, build, key, end, from, flatDistance(from.position, end) * (reverse ? 1.85D : 1.0D), reverse);
    }

    private static void addTrackedPoseNode(Entity vehicle, TrackedPoseRouteBuild build, TrackedPoseKey key, Vec3 position,
                                           TrackedPoseNode parent, double cost, boolean reverse) {
        double total = parent.cost + cost;
        Double previous = build.costs.get(key);
        if (previous != null && previous <= total) {
            pathDebug(vehicle, "POSE_NODE_REJECT", "generation=%d key=(%d,%d,%d) newG=%.3f oldG=%.3f reason=not_better",
                    build.generation, key.x, key.z, key.heading, total, previous);
            return;
        }
        build.costs.put(key, total);
        // Weighted A* keeps the open set aimed at the command target.  With an exact OBB test
        // a broad, unweighted search spent most of its budget rotating through local poses
        // around the first obstacle before it had advanced down either viable side.
        double heuristic = flatDistance(position, build.target);
        TrackedPoseNode opened = new TrackedPoseNode(key, position, parent, total,
                total + heuristic * TRACKED_POSE_HEURISTIC_WEIGHT, reverse);
        build.open.add(opened);
        double bestHeuristic = build.bestNode == null
                ? Double.POSITIVE_INFINITY : flatDistance(build.bestNode.position, build.target);
        if (heuristic < bestHeuristic - 1.0E-6D
                || (Math.abs(heuristic - bestHeuristic) <= 1.0E-6D
                && (build.bestNode == null || total < build.bestNode.cost))) {
            build.bestNode = opened;
        }
        pathDebug(vehicle, "POSE_NODE_OPEN", "generation=%d key=(%d,%d,%d) pos=%s g=%.3f h=%.3f f=%.3f reverse=%s open=%d",
                build.generation, key.x, key.z, key.heading, fmt(position), total, heuristic,
                total + heuristic * TRACKED_POSE_HEURISTIC_WEIGHT, reverse, build.open.size());
    }

    private static List<TrackedPose> reconstructTrackedPoseRoute(TrackedPoseNode end, float startYaw) {
        List<TrackedPose> reversed = new ArrayList<>();
        for (TrackedPoseNode node = end; node != null; node = node.parent) {
            // The search key uses a coarse heading lattice, but the physical tank does not
            // begin at that quantized angle.  Retaining the real start yaw prevents a tank at
            // -17.8 degrees from receiving a fabricated -30 degree pivot before it may climb.
            float yaw = node.parent == null ? startYaw : trackedPoseYaw(node.key.heading);
            reversed.add(new TrackedPose(node.position, yaw, node.reverse));
        }
        Collections.reverse(reversed);
        return reversed;
    }

    /**
     * OBB-aware string pulling.  The grid search is allowed to discover clearance around an
     * obstacle, but its cheap 30-degree lattice must not become the driven route.  From each
     * retained pose, connect to the farthest later pose whose complete pivot and swept hull are
     * valid.  This removes the two-bend doglegs seen on an otherwise clear road while retaining
     * the original escape steps when the vehicle starts embedded beside a wall.
     */
    private static List<TrackedPose> simplifyTrackedPoseRoute(Entity vehicle, List<TrackedPose> raw,
                                                               TrackedHull hull, Set<UUID> ignoredVehicles) {
        if (raw == null || raw.size() < 3) return raw;
        long started = System.nanoTime();
        long deadline = started + TRACKED_POSE_SIMPLIFY_BUDGET_NANOS;
        List<TrackedPose> simplified = new ArrayList<>();
        simplified.add(raw.get(0));
        int anchor = 0;
        while (anchor < raw.size() - 1) {
            if (System.nanoTime() >= deadline) {
                for (int i = anchor + 1; i < raw.size(); i++) appendTrackedPose(simplified, raw.get(i));
                pathDebug(vehicle, "POSE_ROUTE_SIMPLIFY_LIMIT", "rawSteps=%d retainedSteps=%d elapsedMs=%.2f anchor=%d",
                        raw.size(), simplified.size(), (System.nanoTime() - started) / 1_000_000.0D, anchor);
                return simplified;
            }
            TrackedPoseShortcut shortcut = null;
            int destination = raw.size() - 1;
            for (; destination > anchor + 1; destination--) {
                if (System.nanoTime() >= deadline) break;
                shortcut = trackedPoseShortcut(vehicle, raw.get(anchor), raw.get(destination), hull, ignoredVehicles, deadline);
                if (shortcut != null) break;
            }
            if (shortcut == null) {
                appendTrackedPose(simplified, raw.get(anchor + 1));
                pathDebug(vehicle, "POSE_SHORTCUT_KEEP", "from=%d to=%d reason=no_clear_farther_segment",
                        anchor, anchor + 1);
                anchor++;
                continue;
            }
            for (TrackedPose pose : shortcut.steps) appendTrackedPose(simplified, pose);
            pathDebug(vehicle, "POSE_SHORTCUT_APPLY", "from=%d to=%d skipped=%d yaw=%.1f generated=%d",
                    anchor, destination, destination - anchor - 1, shortcut.yaw, shortcut.steps.size());
            anchor = destination;
        }
        pathDebug(vehicle, "POSE_ROUTE_SIMPLIFIED", "rawSteps=%d simplifiedSteps=%d",
                raw.size(), simplified.size());
        return simplified;
    }

    private static TrackedPoseShortcut trackedPoseShortcut(Entity vehicle, TrackedPose from, TrackedPose to,
                                                            TrackedHull hull, Set<UUID> ignoredVehicles, long deadline) {
        Vec3 horizontal = to.position.subtract(from.position).multiply(1.0D, 0.0D, 1.0D);
        if (horizontal.lengthSqr() < 1.0E-6D) return null;
        float yaw = yawTo(horizontal);
        if (!canTrackedPosePivot(vehicle, from.position, from.yaw, yaw, hull, ignoredVehicles)) return null;
        Vec3 reached = sweepTrackedPose(vehicle, from.position, to.position, yaw, hull, ignoredVehicles, false, deadline);
        if (reached == null || flatDistance(reached, to.position) > 0.10D) return null;

        List<TrackedPose> steps = new ArrayList<>();
        if (Math.abs(Mth.wrapDegrees(yaw - from.yaw)) > TRACKED_POSE_YAW_TOLERANCE) {
            steps.add(new TrackedPose(from.position, yaw, false));
        }
        int segments = Math.max(1, Mth.ceil(horizontal.length() / TRACKED_POSE_FORWARD_STEP));
        Vec3 previous = from.position;
        for (int i = 1; i <= segments; i++) {
            if (System.nanoTime() >= deadline) return null;
            double fraction = (double) i / segments;
            Vec3 raw = new Vec3(Mth.lerp(fraction, from.position.x, to.position.x), previous.y,
                    Mth.lerp(fraction, from.position.z, to.position.z));
            Vec3 next = resolveTrackedPosePosition(vehicle, raw, yaw, hull, ignoredVehicles, previous.y);
            if (next == null || !terrainStepAllowed(previous.y, next.y)) return null;
            steps.add(new TrackedPose(next, yaw, false));
            previous = next;
        }
        return new TrackedPoseShortcut(yaw, List.copyOf(steps));
    }

    private static void appendTrackedPose(List<TrackedPose> route, TrackedPose pose) {
        if (route.isEmpty()) {
            route.add(pose);
            return;
        }
        TrackedPose last = route.get(route.size() - 1);
        if (last.position.equals(pose.position)
                && Math.abs(Mth.wrapDegrees(last.yaw - pose.yaw)) <= 0.01F
                && last.reverse == pose.reverse) return;
        route.add(pose);
    }

    private record TrackedPoseShortcut(float yaw, List<TrackedPose> steps) {}

    private static void installTrackedPoseRoute(ServerPlayer player, Entity vehicle, Vec3 target, Vec3 safe,
                                                long generation, TrackedHull hull, List<TrackedPose> steps) {
        if (steps == null || steps.isEmpty()) return;
        TrackedPoseRoute route = new TrackedPoseRoute(target, safe, generation, hull, steps);
        TRACKED_POSE_ROUTES.put(vehicle.getUUID(), route);
        TRACKED_PROXY_DRIVE_UNTIL.remove(vehicle.getUUID());
        CompoundTag data = vehicle.getPersistentData();
        data.putDouble(SAFE_TARGET_X, safe.x);
        data.putDouble(SAFE_TARGET_Y, safe.y);
        data.putDouble(SAFE_TARGET_Z, safe.z);
        data.remove(PATH_BLOCKED);
        List<Vec3> visible = steps.stream().map(TrackedPose::position).toList();
        storeRoute(vehicle, safe, visible, Math.min(1, Math.max(0, visible.size() - 1)));
        refreshPlannedPath(player, vehicle, visible);
        for (int index = 0; index < steps.size(); index++) {
            TrackedPose step = steps.get(index);
            pathDebug(vehicle, "POSE_ROUTE_STEP", "generation=%d index=%d/%d pose=%s yaw=%.1f reverse=%s",
                    generation, index, steps.size(), fmt(step.position), step.yaw, step.reverse);
        }
    }

    private static void failTrackedPoseRoute(Entity vehicle, TrackedPoseRouteBuild build, String reason) {
        TRACKED_POSE_BUILDS.remove(vehicle.getUUID());
        CompoundTag data = vehicle.getPersistentData();
        data.putBoolean(PATH_BLOCKED, true);
        data.putLong(REPLAN_AFTER, vehicle.level().getGameTime() + 60L);
        pathDebug(vehicle, "POSE_ROUTE_BLOCKED", "generation=%d expanded=%d reason=%s target=%s",
                build.generation, build.expanded, reason, fmt(build.target));
        if (!canTrackedPoseOccupy(vehicle, vehicle.position(), vehicle.getYRot(), build.hull,
                Set.of(vehicle.getUUID()))) {
            int nativeStuckTick = vehicle instanceof AbstractVehicle chassis && chassis.physicsEngine != null
                    ? chassis.physicsEngine.stuckTick : 0;
            if (isTrackedPhysicallyStuck(vehicle)) {
                pathDebug(vehicle, "POSE_START_EMBEDDED", "generation=%d expanded=%d pose=%s yaw=%.1f nativeStuckTick=%d action=recovery",
                        build.generation, build.expanded, fmt(vehicle.position()), vehicle.getYRot(), nativeStuckTick);
                beginTrackedRecovery(vehicle, build.target);
            } else {
                // A disagreement between the conservative navigation proxy and native physics
                // is not proof that the tank is wedged.  This most often occurs when the entity
                // centre is over a ditch while the front and rear tracks remain supported.  Do
                // not neutral-lock such a pose: let native physics drive toward the command and
                // let the motion watchdog promote an actual twelve-tick stall to recovery.
                data.putLong(REPLAN_AFTER, vehicle.level().getGameTime() + 8L);
                TRACKED_PROXY_DRIVE_UNTIL.put(vehicle.getUUID(), vehicle.level().getGameTime() + 20L);
                pathDebug(vehicle, "POSE_START_PROXY_CONFLICT", "generation=%d expanded=%d pose=%s yaw=%.1f nativeStuckTick=%d action=native_drive_watchdog",
                        build.generation, build.expanded, fmt(vehicle.position()), vehicle.getYRot(), nativeStuckTick);
            }
        }
    }

    private static boolean moveTrackedPoseRoute(ServerPlayer player, Entity vehicle, Vec3 target, Vec3 safeTarget, VehicleShape shape) {
        TrackedRecovery recovery = TRACKED_RECOVERIES.get(vehicle.getUUID());
        if (recovery == null && isTrackedPhysicallyStuck(vehicle)) {
            recovery = beginTrackedRecovery(vehicle, target);
        }
        if (recovery != null) {
            applyTrackedControl(vehicle, trackedRecoveryControl(vehicle, recovery));
            return true;
        }
        TrackedPoseRouteBuild build = TRACKED_POSE_BUILDS.get(vehicle.getUUID());
        if (build != null) {
            // The planner holds position at its snapshot origin.  For a tracked vehicle an
            // "up" flag is not a brake; native physics brakes positive forward velocity with
            // a backward key, so use the real native braking input here.
            applyTrackedControl(vehicle, trackedBrakeControl(vehicle));
            alignMainWeaponsToYaw(vehicle, trackedWeaponAimYaw(vehicle, null, build, target), true);
            return true;
        }
        TrackedPoseRoute route = TRACKED_POSE_ROUTES.get(vehicle.getUUID());
        if (route == null || !route.matches(target)) {
            if (vehicle.level().getGameTime() >= vehicle.getPersistentData().getLong(REPLAN_AFTER)) {
                prepareTrackedPoseRoute(player, vehicle, target, Set.of(vehicle.getUUID()));
            }
            applyTrackedControl(vehicle, trackedProxyDriveActive(vehicle)
                    ? trackedProxyConflictControl(vehicle, target)
                    : trackedBrakeControl(vehicle));
            alignMainWeaponsToYaw(vehicle, trackedWeaponAimYaw(vehicle, null,
                    TRACKED_POSE_BUILDS.get(vehicle.getUUID()), target), true);
            return true;
        }

        applyTrackedControl(vehicle, trackedPoseControl(vehicle, route));
        alignMainWeaponsToYaw(vehicle, trackedWeaponAimYaw(vehicle, route, null, target), true);
        return true;
    }

    /**
     * Produces exactly one native-tick command for a pose route.  This method is shared by
     * Dominion's command callback and tickGroundControl(), so route progress cannot be skipped
     * while the vehicle is travelling quickly.
     */
    private static GroundControlState trackedPoseControl(Entity vehicle, TrackedPoseRoute route) {
        if (route.stopRequested) {
            if (horizontalSpeed(vehicle) > TRACKED_POSE_PIVOT_SPEED) return trackedBrakeControl(vehicle);
            route.stopRequested = false;
            if (route.replanAfterStop) {
                route.replanAfterStop = false;
                invalidateTrackedPoseRoute(vehicle, route.target, "obstacle_ahead");
                return new GroundControlState(false, false, false, false, false);
            }
        }
        while (route.index < route.steps.size()) {
            TrackedPose step = route.steps.get(route.index);
            TrackedPose previous = route.index > 0 ? route.steps.get(route.index - 1) : null;
            double distance = flatDistance(vehicle.position(), step.position);
            float yawDiff = Mth.wrapDegrees(step.yaw - vehicle.getYRot());
            boolean rotationStep = previous != null && previous.position.equals(step.position);
            // A pure rotation state is complete when its heading is complete.  Its recorded
            // position may be behind the hull after braking, and must not be chased as a point.
            if (rotationStep && Math.abs(yawDiff) <= TRACKED_POSE_YAW_TOLERANCE) {
                route.index++;
                continue;
            }
            if (distance <= TRACKED_POSE_REACH_RADIUS
                    && Math.abs(yawDiff) <= TRACKED_POSE_YAW_TOLERANCE) {
                route.index++;
                continue;
            }
            // A translation node is complete as soon as the hull centre has crossed its
            // end plane.  Do not gate this on yaw: the *next* route state is commonly a
            // pivot, so its yaw intentionally differs exactly when this node must advance.
            // Keeping the old heading gate made the controller keep accelerating towards a
            // node already behind it, which is the turn-then-run-past-the-target failure.
            if (previous != null && !previous.position.equals(step.position)
                    && hasPassedPoint(vehicle.position(), previous.position, step.position)) {
                route.index++;
                continue;
            }
            boolean rollingSteer = trackedShouldRollingSteer(rotationStep, yawDiff);
            if (Math.abs(yawDiff) > TRACKED_POSE_YAW_TOLERANCE && !rollingSteer) {
                // TrackedVehicle preserves its forward velocity when it receives only a
                // left/right key.  A pivot therefore must never begin at road speed: brake
                // first, then rotate after the forward component has genuinely decayed.
                if (horizontalSpeed(vehicle) > TRACKED_POSE_PIVOT_SPEED) return trackedBrakeControl(vehicle);
                if (!canTrackedPosePivot(vehicle, vehicle.position(), vehicle.getYRot(), step.yaw, route.hull(), Set.of(vehicle.getUUID()))) {
                    if (isTrackedPhysicallyStuck(vehicle)) {
                        beginTrackedRecovery(vehicle, route.target);
                        return trackedBrakeControl(vehicle);
                    }
                    // The navigation proxy is deliberately conservative and, on a slope,
                    // its yaw-only hull can overlap terrain already supporting the pitched
                    // physical chassis.  A proxy veto must never deadlock a pose that native
                    // physics still considers movable.  Let native collision resolve the
                    // pivot; the motion watchdog below turns a genuinely failed manoeuvre
                    // into recovery after observing actual lack of movement/rotation.
                    pathDebug(vehicle, "POSE_PIVOT_PROXY_BYPASS",
                            "index=%d yawDiff=%.2f nativeStuckTick=0 action=native_pivot", route.index, yawDiff);
                }
                boolean right = yawDiff > 0.0F;
                return new GroundControlState(false, false, right, !right, false);
            }
            if (distance <= TRACKED_POSE_REACH_RADIUS) {
                route.index++;
                continue;
            }
            Vec3 verified = sweepTrackedPose(vehicle, vehicle.position(), step.position, step.yaw,
                    route.hull(), Set.of(vehicle.getUUID()), step.reverse);
            if (verified == null || flatDistance(verified, step.position) > 0.55D) {
                if (isTrackedPhysicallyStuck(vehicle)) {
                    beginTrackedRecovery(vehicle, route.target);
                } else {
                    route.stopRequested = true;
                    route.replanAfterStop = true;
                }
                return trackedBrakeControl(vehicle);
            }

            double speed = horizontalSpeed(vehicle);
            double clearance = trackedStraightClearance(vehicle, route);
            double stoppingDistance = trackedNativeStoppingDistance(speed);
            // Full throttle is retained until the exact native braking distance of the next
            // pivot/final point.  Once braking starts it is latched until actually stopped;
            // otherwise binary forward/backward controls oscillate and roll through the turn.
            if (clearance <= stoppingDistance + TRACKED_POSE_REACH_RADIUS) {
                route.stopRequested = true;
                return trackedBrakeControl(vehicle);
            }

            // Recheck enough of the already planned straight section to guarantee that the
            // current velocity can stop before a newly placed or previously missed wall.
            double safetyHorizon = Math.min(clearance,
                    stoppingDistance + Math.max(speed, TRACKED_POSE_FORWARD_STEP) + TRACKED_POSE_REACH_RADIUS);
            if (!trackedRouteHorizonClear(vehicle, route, safetyHorizon)) {
                route.stopRequested = true;
                route.replanAfterStop = true;
                return trackedBrakeControl(vehicle);
            }
            boolean steerRight = rollingSteer && (step.reverse ? yawDiff < 0.0F : yawDiff > 0.0F);
            boolean steerLeft = rollingSteer && (step.reverse ? yawDiff > 0.0F : yawDiff < 0.0F);
            if (rollingSteer) {
                pathDebug(vehicle, "POSE_ROLLING_STEER", "index=%d yawDiff=%.2f reverse=%s keys=f:%s,b:%s,r:%s,l:%s",
                        route.index, yawDiff, step.reverse, !step.reverse, step.reverse, steerRight, steerLeft);
            }
            return new GroundControlState(!step.reverse, step.reverse, steerRight, steerLeft, false);
        }
        // Keep this finished route installed as a velocity hold.  Removing it would leave the
        // last forward key latched until the next high-level command callback, which was the
        // source of the long drive past the selected point.
        if (flatDistance(route.safe, route.target) > TRACKED_POSE_GOAL_RADIUS + 0.25D) {
            invalidateTrackedPoseRoute(vehicle, route.target, "partial_complete");
            return new GroundControlState(false, false, false, false, false);
        }
        return trackedBrakeControl(vehicle);
    }

    static boolean trackedShouldRollingSteer(boolean rotationStep, float yawDiff) {
        double absoluteYaw = Math.abs(Mth.wrapDegrees(yawDiff));
        return !rotationStep
                && absoluteYaw > TRACKED_POSE_YAW_TOLERANCE
                && absoluteYaw <= TRACKED_POSE_ROLLING_STEER_MAX_YAW;
    }

    private static boolean trackedProxyDriveActive(Entity vehicle) {
        Long until = TRACKED_PROXY_DRIVE_UNTIL.get(vehicle.getUUID());
        if (until == null) return false;
        if (vehicle.level().getGameTime() <= until) return true;
        TRACKED_PROXY_DRIVE_UNTIL.remove(vehicle.getUUID(), until);
        return false;
    }

    /** Native-control escape for a pose rejected only by the conservative planning proxy. */
    private static GroundControlState trackedProxyConflictControl(Entity vehicle, Vec3 target) {
        if (target == null) return trackedBrakeControl(vehicle);
        Vec3 delta = target.subtract(vehicle.position()).multiply(1.0D, 0.0D, 1.0D);
        double distance = delta.length();
        if (distance <= TRACKED_POSE_REACH_RADIUS || delta.lengthSqr() < 1.0E-6D) {
            return trackedBrakeControl(vehicle);
        }
        float targetYaw = yawTo(delta);
        float yawDiff = Mth.wrapDegrees(targetYaw - vehicle.getYRot());
        boolean reverse = trackedShouldDirectShortReverse(vehicle.getYRot(), targetYaw, distance);
        if (!reverse && Math.abs(yawDiff) > TRACKED_POSE_ROLLING_STEER_MAX_YAW) {
            if (horizontalSpeed(vehicle) > TRACKED_POSE_PIVOT_SPEED) return trackedBrakeControl(vehicle);
            boolean right = yawDiff > 0.0F;
            pathDebug(vehicle, "POSE_PROXY_NATIVE_DRIVE", "mode=pivot target=%s distance=%.2f yawDiff=%.1f keys=r:%s,l:%s",
                    fmt(target), distance, yawDiff, right, !right);
            return new GroundControlState(false, false, right, !right, false);
        }
        boolean right = reverse ? yawDiff < 0.0F : yawDiff > TRACKED_POSE_YAW_TOLERANCE;
        boolean left = reverse ? yawDiff > 0.0F : yawDiff < -TRACKED_POSE_YAW_TOLERANCE;
        pathDebug(vehicle, "POSE_PROXY_NATIVE_DRIVE", "mode=%s target=%s distance=%.2f yawDiff=%.1f keys=f:%s,b:%s,r:%s,l:%s",
                reverse ? "reverse" : "forward", fmt(target), distance, yawDiff, !reverse, reverse, right, left);
        return new GroundControlState(!reverse, reverse, right, left, false);
    }

    private static void applyTrackedControl(Entity vehicle, GroundControlState control) {
        applyControl(vehicle, control.forward(), control.backward(), control.right(), control.left(), control.brake());
    }

    /** Native TrackedVehicle ignores control.up; backward is its positive-speed brake. */
    private static GroundControlState trackedBrakeControl(Entity vehicle) {
        Vec3 velocity = vehicle.getDeltaMovement().multiply(1.0D, 0.0D, 1.0D);
        if (velocity.lengthSqr() <= 6.25E-4D) return new GroundControlState(false, false, false, false, false);
        Vec3 look = vehicle.getLookAngle().multiply(1.0D, 0.0D, 1.0D);
        boolean movingForward = look.lengthSqr() < 1.0E-6D || velocity.normalize().dot(look.normalize()) >= 0.0D;
        return new GroundControlState(!movingForward, movingForward, false, false, false);
    }

    /** Exact discrete stopping distance produced by TrackedVehicle's 0.025/tick opposite key. */
    private static double trackedNativeStoppingDistance(double speed) {
        double remaining = Math.max(0.0D, speed);
        double distance = 0.0D;
        while (remaining > 1.0E-6D) {
            remaining = Math.max(0.0D, remaining - TRACKED_NATIVE_BRAKE_PER_TICK);
            distance += remaining;
        }
        return distance;
    }

    /** Distance along the current straight/reverse run before a pivot, direction change, or end. */
    private static double trackedStraightClearance(Entity vehicle, TrackedPoseRoute route) {
        if (route.index >= route.steps.size()) return 0.0D;
        TrackedPose first = route.steps.get(route.index);
        Vec3 previous = vehicle.position();
        double distance = 0.0D;
        for (int index = route.index; index < route.steps.size(); index++) {
            TrackedPose step = route.steps.get(index);
            if (step.reverse != first.reverse
                    || Math.abs(Mth.wrapDegrees(step.yaw - first.yaw)) > TRACKED_POSE_YAW_TOLERANCE) break;
            distance += flatDistance(previous, step.position);
            previous = step.position;
        }
        return distance;
    }

    /** Revalidates the route from the actual pose, not merely from its old planned grid nodes. */
    private static boolean trackedRouteHorizonClear(Entity vehicle, TrackedPoseRoute route, double horizon) {
        if (horizon <= 1.0E-4D || route.index >= route.steps.size()) return true;
        TrackedPose first = route.steps.get(route.index);
        Vec3 previous = vehicle.position();
        double remaining = horizon;
        for (int index = route.index; index < route.steps.size() && remaining > 1.0E-4D; index++) {
            TrackedPose step = route.steps.get(index);
            if (step.reverse != first.reverse
                    || Math.abs(Mth.wrapDegrees(step.yaw - first.yaw)) > TRACKED_POSE_YAW_TOLERANCE) break;
            double segment = flatDistance(previous, step.position);
            if (segment <= 1.0E-6D) {
                previous = step.position;
                continue;
            }
            Vec3 target = segment <= remaining
                    ? step.position
                    : previous.add(step.position.subtract(previous).scale(remaining / segment));
            Vec3 reached = sweepTrackedPose(vehicle, previous, target, step.yaw,
                    route.hull(), Set.of(vehicle.getUUID()), step.reverse);
            if (reached == null || flatDistance(reached, target) > TRACKED_POSE_REACH_RADIUS) return false;
            remaining -= Math.min(segment, remaining);
            previous = target;
        }
        return true;
    }

    private static boolean isTrackedPhysicallyStuck(Entity vehicle) {
        return vehicle instanceof AbstractVehicle chassis
                && chassis.physicsEngine != null
                && chassis.physicsEngine.stuckTick >= 2;
    }

    private static Vec3 trackedStoredTarget(Entity vehicle) {
        CompoundTag data = vehicle.getPersistentData();
        if (data.contains(FINAL_TARGET_X) && data.contains(FINAL_TARGET_Z)) {
            return new Vec3(data.getDouble(FINAL_TARGET_X), vehicle.getY(), data.getDouble(FINAL_TARGET_Z));
        }
        return vehicle.position();
    }

    private static TrackedRecovery beginTrackedRecovery(Entity vehicle, Vec3 target) {
        TrackedRecovery existing = TRACKED_RECOVERIES.get(vehicle.getUUID());
        if (existing != null) return existing;
        TRACKED_POSE_ROUTES.remove(vehicle.getUUID());
        TRACKED_POSE_BUILDS.remove(vehicle.getUUID());
        TrackedHull hull = vehicle instanceof AbstractVehicle chassis ? TrackedHull.from(chassis) : null;
        TrackedRecovery recovery = new TrackedRecovery(vehicle.position(), target == null ? trackedStoredTarget(vehicle) : target, hull);
        TRACKED_RECOVERIES.put(vehicle.getUUID(), recovery);
        pathDebug(vehicle, "POSE_RECOVERY_START", "stuckTick=%d target=%s",
                vehicle instanceof AbstractVehicle chassis && chassis.physicsEngine != null ? chassis.physicsEngine.stuckTick : 0,
                fmt(recovery.target));
        return recovery;
    }

    /** Brake, reverse clear of the contact, brake again, then permit a fresh pose search. */
    private static GroundControlState trackedRecoveryControl(Entity vehicle, TrackedRecovery recovery) {
        long gameTime = vehicle.level().getGameTime();
        boolean newTick = recovery.lastTick != gameTime;
        if (newTick) {
            recovery.lastTick = gameTime;
            recovery.ticks++;
        }
        double speed = horizontalSpeed(vehicle);
        if (recovery.brakingBeforeReverse) {
            if (speed > TRACKED_POSE_PIVOT_SPEED) return trackedBrakeControl(vehicle);
            recovery.brakingBeforeReverse = false;
            recovery.reverseOrigin = vehicle.position();
        }
        if (recovery.brakingAfterReverse) {
            if (speed > TRACKED_POSE_PIVOT_SPEED) return trackedBrakeControl(vehicle);
            TRACKED_RECOVERIES.remove(vehicle.getUUID());
            if (vehicle instanceof AbstractVehicle chassis && chassis.physicsEngine != null) chassis.physicsEngine.stuckTick = 0;
            CompoundTag data = vehicle.getPersistentData();
            data.remove(PATH_BLOCKED);
            data.putLong(REPLAN_AFTER, gameTime);
            pathDebug(vehicle, "POSE_RECOVERY_DONE", "moved=%.2f ticks=%d target=%s",
                    flatDistance(recovery.reverseOrigin, vehicle.position()), recovery.ticks, fmt(recovery.target));
            return new GroundControlState(false, false, false, false, false);
        }

        if (newTick) recovery.reverseTicks++;
        double moved = flatDistance(recovery.reverseOrigin, vehicle.position());
        if (recovery.reverseTicks >= 24 && moved < 0.20D && recovery.hull != null
                && vehicle instanceof AbstractVehicle chassis) {
            Vec3 escape = findTrackedDepenetrationPose(vehicle, recovery);
            if (escape != null) {
                Vec3 before = vehicle.position();
                vehicle.setPos(escape.x, escape.y, escape.z);
                vehicle.setDeltaMovement(Vec3.ZERO);
                chassis.updateOBBs();
                if (chassis.physicsEngine != null) chassis.physicsEngine.stuckTick = 0;
                recovery.reverseOrigin = escape;
                recovery.brakingAfterReverse = true;
                pathDebug(vehicle, "POSE_RECOVERY_DEPENETRATE", "from=%s to=%s distance=%.3f reverseTicks=%d",
                        fmt(before), fmt(escape), flatDistance(before, escape), recovery.reverseTicks);
                return new GroundControlState(false, false, false, false, false);
            }
            pathDebug(vehicle, "POSE_RECOVERY_DEPENETRATE_FAILED", "origin=%s yaw=%.1f reverseTicks=%d",
                    fmt(vehicle.position()), vehicle.getYRot(), recovery.reverseTicks);
        }
        if (moved >= TRACKED_RECOVERY_DISTANCE || recovery.ticks >= TRACKED_RECOVERY_MAX_TICKS) {
            recovery.brakingAfterReverse = true;
            return trackedBrakeControl(vehicle);
        }

        // A straight reverse clears a frontal collision.  If it has made no useful progress,
        // alternate a reverse steering input so a corner/side contact cannot lock controls.
        boolean steer = recovery.reverseTicks > 16 && moved < 0.35D;
        boolean right = ((recovery.reverseTicks / 12) & 1) == 0;
        return new GroundControlState(false, true, steer && right, steer && !right, false);
    }

    private static Vec3 findTrackedDepenetrationPose(Entity vehicle, TrackedRecovery recovery) {
        Vec3 origin = vehicle.position();
        Vec3 forward = vehicle.getLookAngle().multiply(1.0D, 0.0D, 1.0D);
        if (forward.lengthSqr() < 1.0E-6D) forward = Vec3.directionFromRotation(0.0F, vehicle.getYRot());
        forward = forward.normalize();
        Vec3 backward = forward.scale(-1.0D);
        Vec3 right = new Vec3(-forward.z, 0.0D, forward.x);
        double[] lateralOffsets = {0.0D, 0.25D, -0.25D, 0.50D, -0.50D, 0.75D, -0.75D, 1.0D, -1.0D};
        for (double distance = 0.25D; distance <= 4.0D; distance += 0.25D) {
            for (double lateral : lateralOffsets) {
                Vec3 raw = origin.add(backward.scale(distance)).add(right.scale(lateral));
                Vec3 candidate = resolveTrackedPosePosition(vehicle, raw, vehicle.getYRot(), recovery.hull,
                        Set.of(vehicle.getUUID()), origin.y);
                pathDebug(vehicle, candidate == null ? "POSE_DEPENETRATE_CANDIDATE_REJECT" : "POSE_DEPENETRATE_CANDIDATE_PASS",
                        "raw=%s distance=%.2f lateral=%.2f resolved=%s", fmt(raw), distance, lateral, fmt(candidate));
                if (candidate != null) return candidate;
            }
        }
        for (double distance = 0.50D; distance <= 4.0D; distance += 0.50D) {
            for (int angle = 0; angle < 360; angle += 15) {
                double radians = Math.toRadians(angle);
                Vec3 raw = origin.add(Math.cos(radians) * distance, 0.0D, Math.sin(radians) * distance);
                Vec3 candidate = resolveTrackedPosePosition(vehicle, raw, vehicle.getYRot(), recovery.hull,
                        Set.of(vehicle.getUUID()), origin.y);
                pathDebug(vehicle, candidate == null ? "POSE_DEPENETRATE_CANDIDATE_REJECT" : "POSE_DEPENETRATE_CANDIDATE_PASS",
                        "raw=%s radius=%.2f angle=%d resolved=%s fallback=radial", fmt(raw), distance, angle, fmt(candidate));
                if (candidate != null) return candidate;
            }
        }
        return null;
    }

    private static void traceTrackedPoseTick(Entity vehicle, AbstractVehicle chassis, TrackedPoseRoute route,
                                             TrackedRecovery recovery, GroundControlState control) {
        int stuckTick = chassis.physicsEngine == null ? 0 : chassis.physicsEngine.stuckTick;
        if (recovery != null) {
            String stage = recovery.brakingBeforeReverse ? "brake_before_reverse"
                    : recovery.brakingAfterReverse ? "brake_after_reverse" : "reverse_escape";
            pathDebug(vehicle, "POSE_CONTROL_TICK", "mode=recovery stage=%s ticks=%d reverseTicks=%d moved=%.3f stuckTick=%d power=%.1f keys=f:%s,b:%s,r:%s,l:%s",
                    stage, recovery.ticks, recovery.reverseTicks,
                    flatDistance(recovery.reverseOrigin, vehicle.position()), stuckTick, chassis.getPower(),
                    control.forward(), control.backward(), control.right(), control.left());
            return;
        }
        if (route == null || route.index >= route.steps.size()) {
            pathDebug(vehicle, "POSE_CONTROL_TICK", "mode=%s index=%d total=%d stuckTick=%d power=%.1f keys=f:%s,b:%s,r:%s,l:%s",
                    route == null ? "no_route" : "route_finished", route == null ? -1 : route.index,
                    route == null ? 0 : route.steps.size(), stuckTick, chassis.getPower(),
                    control.forward(), control.backward(), control.right(), control.left());
            return;
        }
        TrackedPose step = route.steps.get(route.index);
        TrackedPose previous = route.index > 0 ? route.steps.get(route.index - 1) : null;
        boolean rotation = previous != null && previous.position.equals(step.position);
        double distance = flatDistance(vehicle.position(), step.position);
        double yawDiff = Mth.wrapDegrees(step.yaw - vehicle.getYRot());
        double speed = horizontalSpeed(vehicle);
        double stopping = trackedNativeStoppingDistance(speed);
        double clearance = rotation ? 0.0D : trackedStraightClearance(vehicle, route);
        pathDebug(vehicle, "POSE_CONTROL_TICK", "mode=route generation=%d index=%d/%d step=%s stepYaw=%.1f reverse=%s rotation=%s distance=%.3f yawDiff=%.2f speed=%.3f stopDistance=%.3f clearance=%.3f stopRequested=%s replanAfterStop=%s stuckTick=%d power=%.1f keys=f:%s,b:%s,r:%s,l:%s",
                route.generation, route.index, route.steps.size(), fmt(step.position), step.yaw, step.reverse,
                rotation, distance, yawDiff, speed, stopping, clearance, route.stopRequested,
                route.replanAfterStop, stuckTick, chassis.getPower(), control.forward(), control.backward(), control.right(), control.left());
    }

    private static void watchTrackedMotion(Entity vehicle, AbstractVehicle chassis, TrackedPoseRoute route,
                                           TrackedRecovery recovery, GroundControlState control) {
        long now = vehicle.level().getGameTime();
        TrackedMotionWatch watch = TRACKED_MOTION_WATCHES.computeIfAbsent(vehicle.getUUID(),
                ignored -> new TrackedMotionWatch(vehicle.position(), vehicle.getYRot(), now));
        if (watch.lastTick == now) return;
        double moved = flatDistance(watch.lastPosition, vehicle.position());
        double yawMoved = Math.abs(Mth.wrapDegrees(vehicle.getYRot() - watch.lastYaw));
        boolean commandedMotion = recovery == null
                && (((control.forward() || control.backward()) && chassis.getPower() > 20.0F)
                || control.right() || control.left());
        if (commandedMotion && moved < 0.015D && yawMoved < 0.35D) watch.stalledTicks++;
        else watch.stalledTicks = 0;
        watch.lastPosition = vehicle.position();
        watch.lastYaw = vehicle.getYRot();
        watch.lastTick = now;
        pathDebug(vehicle, "POSE_MOTION_WATCH", "commandedMotion=%s moved=%.4f yawMoved=%.3f stalledTicks=%d stuckTick=%d power=%.1f",
                commandedMotion, moved, yawMoved, watch.stalledTicks,
                chassis.physicsEngine == null ? 0 : chassis.physicsEngine.stuckTick, chassis.getPower());
        if (watch.stalledTicks >= 12 && !TRACKED_RECOVERIES.containsKey(vehicle.getUUID())) {
            pathDebug(vehicle, "POSE_STALL_DETECTED", "moved=%.4f yawMoved=%.3f stalledTicks=%d nativeStuckTick=%d",
                    moved, yawMoved, watch.stalledTicks, chassis.physicsEngine == null ? 0 : chassis.physicsEngine.stuckTick);
            beginTrackedRecovery(vehicle, route == null ? trackedStoredTarget(vehicle) : route.target);
            watch.stalledTicks = 0;
        }
    }

    private static void invalidateTrackedPoseRoute(Entity vehicle, Vec3 target, String reason) {
        TRACKED_POSE_ROUTES.remove(vehicle.getUUID());
        CompoundTag data = vehicle.getPersistentData();
        // trackedPoseReplanCoolingDown checks PATH_BLOCKED as well as REPLAN_AFTER.  Omitting
        // this flag made the four-tick cooldown a no-op and caused hundreds of identical
        // route-build/pivot-reject cycles while the tank remained stationary.
        data.putBoolean(PATH_BLOCKED, true);
        data.putLong(REPLAN_AFTER, vehicle.level().getGameTime() + 4L);
        pathDebug(vehicle, "POSE_ROUTE_INVALID", "reason=%s target=%s", reason, fmt(target));
    }

    private static boolean canTrackedPosePivot(Entity vehicle, Vec3 position, float fromYaw, float toYaw,
                                                TrackedHull hull, Set<UUID> ignoredVehicles) {
        float diff = Mth.wrapDegrees(toYaw - fromYaw);
        int samples = Math.max(1, Mth.ceil(Math.abs(diff) / 5.0F));
        for (int i = 1; i <= samples; i++) {
            float yaw = Mth.wrapDegrees(fromYaw + diff * i / samples);
            if (!canTrackedPoseOccupy(vehicle, position, yaw, hull, ignoredVehicles)) return false;
        }
        return true;
    }

    /** Returns the reached, terrain-following end position, or null when any short pose is impossible. */
    private static Vec3 sweepTrackedPose(Entity vehicle, Vec3 from, Vec3 target, float yaw, TrackedHull hull,
                                         Set<UUID> ignoredVehicles, boolean reverse) {
        return sweepTrackedPose(vehicle, from, target, yaw, hull, ignoredVehicles, reverse, Long.MAX_VALUE);
    }

    private static Vec3 sweepTrackedPose(Entity vehicle, Vec3 from, Vec3 target, float yaw, TrackedHull hull,
                                         Set<UUID> ignoredVehicles, boolean reverse, long deadlineNanos) {
        TrackedPoseRouteBuild activeBuild = vehicle == null ? null : TRACKED_POSE_BUILDS.get(vehicle.getUUID());
        if (activeBuild != null) activeBuild.sweeps++;
        double distance = flatDistance(from, target);
        if (distance < 1.0E-6D) return resolveTrackedPosePosition(vehicle, target, yaw, hull, ignoredVehicles, from.y);
        int samples = Math.max(1, Mth.ceil(distance / 0.25D));
        Vec3 previous = from;
        for (int i = 1; i <= samples; i++) {
            if (System.nanoTime() >= deadlineNanos) return null;
            double t = (double) i / samples;
            Vec3 raw = new Vec3(Mth.lerp(t, from.x, target.x), previous.y, Mth.lerp(t, from.z, target.z));
            Vec3 next = resolveTrackedPosePosition(vehicle, raw, yaw, hull, ignoredVehicles, previous.y);
            if (next == null) {
                pathDebug(vehicle, "POSE_SWEEP_REJECT", "from=%s target=%s yaw=%.1f reverse=%s sample=%d/%d raw=%s previous=%s reason=no_pose",
                        fmt(from), fmt(target), yaw, reverse, i, samples, fmt(raw), fmt(previous));
                return null;
            }
            if (!terrainStepAllowed(previous.y, next.y)) {
                pathDebug(vehicle, "POSE_SWEEP_REJECT", "from=%s target=%s yaw=%.1f reverse=%s sample=%d/%d previous=%s next=%s dy=%.3f maxDy=%.3f reason=step_height",
                        fmt(from), fmt(target), yaw, reverse, i, samples, fmt(previous), fmt(next), next.y - previous.y, MAX_TRAVEL_STEP_HEIGHT);
                return null;
            }
            previous = next;
        }
        pathDebug(vehicle, "POSE_SWEEP_PASS", "from=%s target=%s resolved=%s yaw=%.1f reverse=%s samples=%d",
                fmt(from), fmt(target), fmt(previous), yaw, reverse, samples);
        return previous;
    }

    private static Vec3 resolveTrackedPosePosition(Entity vehicle, Vec3 around, float yaw, TrackedHull hull,
                                                    Set<UUID> ignoredVehicles, double referenceY) {
        if (!(vehicle.level() instanceof ServerLevel level)) return around;
        // A long chassis may bridge a narrow gap with no floor below its entity centre.  Preserve
        // the previous support plane first; the footprint support probes below decide whether
        // enough of the actual tracks are still grounded.
        Vec3 bridged = new Vec3(around.x, referenceY, around.z);
        if (canTrackedPoseOccupy(vehicle, bridged, yaw, hull, ignoredVehicles)) {
            pathDebug(vehicle, "POSE_RESOLVE_PASS", "around=%s referenceY=%.3f candidate=%s yaw=%.1f source=footprint_bridge",
                    fmt(around), referenceY, fmt(bridged), yaw);
            return bridged;
        }
        BlockPos base = BlockPos.containing(around.x, referenceY, around.z);
        for (int dy = 2; dy >= -4; dy--) {
            BlockPos floor = base.offset(0, dy - 1, 0);
            if (!level.getBlockState(floor).isSolid()) continue;
            Vec3 candidate = new Vec3(around.x, floor.getY() + 1.0D + hull.entityGroundOffset, around.z);
            if (canTrackedPoseOccupy(vehicle, candidate, yaw, hull, ignoredVehicles)) {
                pathDebug(vehicle, "POSE_RESOLVE_PASS", "around=%s referenceY=%.3f floor=(%d,%d,%d) floorState=%s groundOffset=%.3f candidate=%s yaw=%.1f",
                        fmt(around), referenceY, floor.getX(), floor.getY(), floor.getZ(),
                        level.getBlockState(floor).getBlock(), hull.entityGroundOffset, fmt(candidate), yaw);
                return candidate;
            }
            pathDebug(vehicle, "POSE_RESOLVE_TRY_REJECT", "around=%s referenceY=%.3f floor=(%d,%d,%d) floorState=%s groundOffset=%.3f candidate=%s yaw=%.1f",
                    fmt(around), referenceY, floor.getX(), floor.getY(), floor.getZ(),
                    level.getBlockState(floor).getBlock(), hull.entityGroundOffset, fmt(candidate), yaw);
        }
        int terrainY = level.getHeight(Heightmap.Types.MOTION_BLOCKING_NO_LEAVES, Mth.floor(around.x), Mth.floor(around.z));
        double entityY = terrainY + hull.entityGroundOffset;
        if (Math.abs(entityY - referenceY) > 1.0E-6D) {
            Vec3 candidate = new Vec3(around.x, entityY, around.z);
            if (canTrackedPoseOccupy(vehicle, candidate, yaw, hull, ignoredVehicles)) {
                pathDebug(vehicle, "POSE_RESOLVE_PASS", "around=%s referenceY=%.3f heightmapY=%d groundOffset=%.3f candidate=%s yaw=%.1f source=heightmap",
                        fmt(around), referenceY, terrainY, hull.entityGroundOffset, fmt(candidate), yaw);
                return candidate;
            }
        }
        pathDebug(vehicle, "POSE_RESOLVE_REJECT", "around=%s referenceY=%.3f yaw=%.1f base=(%d,%d,%d) heightmapY=%d groundOffset=%.3f reason=no_occupiable_height",
                fmt(around), referenceY, yaw, base.getX(), base.getY(), base.getZ(), terrainY, hull.entityGroundOffset);
        return null;
    }

    /**
     * Exact YWZJ compatible pose test.  The broad AABB is deliberately first: if that outer
     * box is clear, the OBB it contains is certainly clear.  Only a broad-phase hit pays for
     * the native main-cube surface sampling, where ground contact is distinguished from a wall.
     */
    private static boolean canTrackedPoseOccupy(Entity vehicle, Vec3 position, float yaw, TrackedHull hull,
                                                Set<UUID> ignoredVehicles) {
        if (!(vehicle instanceof AbstractVehicle chassis) || !(vehicle.level() instanceof ServerLevel level)) return true;
        TrackedPoseRouteBuild activeBuild = TRACKED_POSE_BUILDS.get(vehicle.getUUID());
        if (activeBuild != null) activeBuild.poseTests++;
        TrackedPoseCacheKey cacheKey = activeBuild == null ? null : trackedPoseCacheKey(position, yaw);
        if (cacheKey != null) {
            Boolean cached = activeBuild.poseCache.get(cacheKey);
            if (cached != null) {
                activeBuild.cacheHits++;
                return cached;
            }
        }
        TrackedPoseTransform transform = hull.transform(chassis, position, yaw);
        AABB bounds = transform.bounds();
        int supportMask = trackedPoseSupportMask(level, transform);
        if (!trackedSupportPatternAccepts((supportMask & 1) != 0, (supportMask & 2) != 0,
                (supportMask & 4) != 0, (supportMask & 8) != 0, (supportMask & 16) != 0)) {
            if (activeBuild != null) activeBuild.rejectedPoses++;
            pathDebug(vehicle, "POSE_OCCUPY_REJECT", "pose=%s yaw=%.1f bounds=%s supportMask=0x%02X reason=no_footprint_support",
                    fmt(position), yaw, boxBounds(bounds), supportMask);
            return cacheTrackedPoseResult(activeBuild, cacheKey, false);
        }

        // Lift the broad box a hair off the supporting ground.  Native physics permits that
        // contact; it is not an obstacle to driving or pivoting.
        AABB broadphase = new AABB(bounds.minX, bounds.minY + 0.035D, bounds.minZ,
                bounds.maxX, bounds.maxY, bounds.maxZ);
        boolean clearAabb = level.noCollision(vehicle, broadphase);
        if (!clearAabb && activeBuild != null) activeBuild.obbFallbacks++;
        BlockPos blockingBlock = clearAabb ? null : trackedPoseConvexBlockingBlock(level, transform);
        if (blockingBlock != null) {
            if (activeBuild != null) {
                activeBuild.rejectedPoses++;
                activeBuild.convexRejects++;
            }
            pathDebug(vehicle, "POSE_OCCUPY_REJECT", "pose=%s yaw=%.1f bounds=%s broadphase=%s block=(%d,%d,%d) blockState=%s lowSideY=%.3f reason=convex_sat",
                    fmt(position), yaw, boxBounds(bounds), boxBounds(broadphase),
                    blockingBlock.getX(), blockingBlock.getY(), blockingBlock.getZ(),
                    level.getBlockState(blockingBlock).getBlock(), hull.lowSideContactY);
            return cacheTrackedPoseResult(activeBuild, cacheKey, false);
        }

        for (Entity other : level.getEntities(vehicle, bounds.inflate(0.05D))) {
            if (other == vehicle || !isInstance(other, VEHICLE_CLASS_NAME)) continue;
            if (ignoredVehicles != null && ignoredVehicles.contains(other.getUUID())) continue;
            if (other.getBoundingBox().intersects(bounds.inflate(0.10D))) {
                if (activeBuild != null) activeBuild.rejectedPoses++;
                pathDebug(vehicle, "POSE_OCCUPY_REJECT", "pose=%s yaw=%.1f bounds=%s other=%s otherBox=%s reason=vehicle",
                        fmt(position), yaw, boxBounds(bounds), other.getStringUUID(), boxBounds(other.getBoundingBox()));
                return cacheTrackedPoseResult(activeBuild, cacheKey, false);
            }
        }
        // Do not use a heightmap over the whole projected hull here.  A house wall or roof
        // beside a clear corridor appears as a tall "ground" column in that map, which made
        // every first move near a building fail even though YWZJ's own OBB samples found no
        // contact.  The centre support probe above plus the native-equivalent surface samples
        // are the actual traversability authority; sweepTrackedPose still rejects a wall-sized
        // change along the direction of travel.
        pathDebug(vehicle, "POSE_OCCUPY_PASS", "pose=%s yaw=%.1f bounds=%s broadphase=%s mode=%s supportMask=0x%02X groundOffset=%.3f",
                fmt(position), yaw, boxBounds(bounds), boxBounds(broadphase), clearAabb ? "aabb" : "obb_fallback",
                supportMask, hull.entityGroundOffset);
        return cacheTrackedPoseResult(activeBuild, cacheKey, true);
    }

    /** centre/front/rear/left/right bits for support under the yaw-only hull footprint. */
    private static int trackedPoseSupportMask(ServerLevel level, TrackedPoseTransform transform) {
        float bottom = -transform.hull.extents.y + 0.05F;
        float x = transform.hull.extents.x * 0.72F;
        float z = transform.hull.extents.z * 0.72F;
        Vector3f[] probes = {
                new Vector3f(0.0F, bottom, 0.0F),
                new Vector3f(0.0F, bottom, z),
                new Vector3f(0.0F, bottom, -z),
                new Vector3f(x, bottom, 0.0F),
                new Vector3f(-x, bottom, 0.0F)
        };
        int mask = 0;
        for (int i = 0; i < probes.length; i++) {
            if (trackedSupportProbe(level, transform.world(probes[i]))) mask |= 1 << i;
        }
        return mask;
    }

    private static boolean trackedSupportProbe(ServerLevel level, Vec3 probe) {
        int minY = Mth.floor(probe.y - 1.05D);
        int maxY = Mth.floor(probe.y + MAX_TRAVEL_STEP_HEIGHT + 0.05D);
        int x = Mth.floor(probe.x), z = Mth.floor(probe.z);
        for (int y = minY; y <= maxY; y++) {
            BlockPos block = new BlockPos(x, y, z);
            VoxelShape shape = level.getBlockState(block).getCollisionShape(level, block);
            if (shape.isEmpty()) continue;
            for (AABB local : shape.toAabbs()) {
                AABB world = local.move(block);
                if (world.minY <= probe.y + 0.08D
                        && world.maxY >= probe.y - 0.15D
                        && world.maxY <= probe.y + MAX_TRAVEL_STEP_HEIGHT + 0.08D) return true;
            }
        }
        return false;
    }

    static boolean trackedSupportPatternAccepts(boolean center, boolean front, boolean rear,
                                                boolean left, boolean right) {
        if (center || front && rear || left && right) return true;
        int peripheral = (front ? 1 : 0) + (rear ? 1 : 0) + (left ? 1 : 0) + (right ? 1 : 0);
        return peripheral >= 2;
    }

    private static boolean cacheTrackedPoseResult(TrackedPoseRouteBuild build, TrackedPoseCacheKey key, boolean result) {
        if (build != null && key != null) build.poseCache.put(key, result);
        return result;
    }

    /** Half-block position cache; yaw remains at five-degree precision for direct shortcuts. */
    private static TrackedPoseCacheKey trackedPoseCacheKey(Vec3 position, float yaw) {
        return new TrackedPoseCacheKey((int) Math.round(position.x * 2.0D),
                (int) Math.round(position.y * 2.0D),
                (int) Math.round(position.z * 2.0D),
                Math.round(Mth.wrapDegrees(yaw) / 5.0F));
    }

    /**
     * Intermediate collision proxy used only after the broad AABB reports a hit.  It is the
     * upper, wall-sensitive portion of the main vehicle cube represented by eight vertices.
     * Minecraft collision shapes are unions of AABBs, so the existing JOML 15-axis SAT gives
     * an exact prism-vs-shape result without transforming every native surface probe.
     */
    private static BlockPos trackedPoseConvexBlockingBlock(ServerLevel level, TrackedPoseTransform transform) {
        OBB prism = transform.navigationPrism();
        AABB scan = trackedObbBounds(prism);
        int minX = Mth.floor(scan.minX), maxX = Mth.floor(scan.maxX - 1.0E-6D);
        int minY = Mth.floor(scan.minY), maxY = Mth.floor(scan.maxY - 1.0E-6D);
        int minZ = Mth.floor(scan.minZ), maxZ = Mth.floor(scan.maxZ - 1.0E-6D);
        for (BlockPos block : BlockPos.betweenClosed(minX, minY, minZ, maxX, maxY, maxZ)) {
            BlockState state = level.getBlockState(block);
            VoxelShape shape = state.getCollisionShape(level, block);
            if (shape.isEmpty()) continue;
            for (AABB local : shape.toAabbs()) {
                AABB worldShape = local.move(block);
                // When the centre probe steps down, the long hull still bridges the preceding
                // higher block.  Terrain connected to the candidate base and no taller than
                // the declared one-block step is support, not a wall.  Floating shapes and
                // taller obstacles continue into the SAT test below.
                VoxelShape aboveShape = level.getBlockState(block.above()).getCollisionShape(level, block.above());
                double horizontalDistance = Math.hypot(block.getX() + 0.5D - transform.center.x,
                        block.getZ() + 0.5D - transform.center.z);
                if (trackedTerrainContact(transform.entityY, transform.hull.entityGroundOffset, worldShape)
                        || trackedExposedTerrainContact(transform.entityY, transform.hull.entityGroundOffset,
                        worldShape, horizontalDistance, aboveShape.isEmpty())) {
                    TrackedPoseRouteBuild active = TRACKED_POSE_BUILDS.get(transform.vehicleId);
                    if (active != null) active.terrainContacts++;
                    continue;
                }
                if (OBB.isColliding(prism, worldShape)) return block.immutable();
            }
        }
        return null;
    }

    static boolean trackedTerrainContact(double entityY, double entityGroundOffset, AABB worldShape) {
        double supportSurfaceY = entityY - entityGroundOffset;
        return worldShape != null
                && worldShape.minY <= supportSurfaceY + 0.035D
                && worldShape.maxY <= supportSurfaceY + MAX_TRAVEL_STEP_HEIGHT + 0.035D;
    }

    /**
     * A long tracked hull spans several stair-step blocks on a slope.  Relative to the centre
     * support, its nose can legitimately touch terrain more than one block higher.  Treat the
     * exposed top of a column as terrain inside a one-block-per-horizontal-block climb
     * envelope.  A wall/tree remains blocking because its lower colliding column has another
     * collision shape above it and therefore is not an exposed driving surface.
     */
    static boolean trackedExposedTerrainContact(double entityY, double entityGroundOffset, AABB worldShape,
                                                double horizontalDistance, boolean exposedTop) {
        if (worldShape == null || !exposedTop || !Double.isFinite(horizontalDistance)) return false;
        double supportSurfaceY = entityY - entityGroundOffset;
        double climbEnvelope = supportSurfaceY + MAX_TRAVEL_STEP_HEIGHT
                + Math.max(0.0D, horizontalDistance) * MAX_TRAVEL_STEP_HEIGHT;
        return worldShape.maxY <= climbEnvelope + 0.035D;
    }

    private static AABB trackedObbBounds(OBB obb) {
        double minX = Double.POSITIVE_INFINITY, minY = Double.POSITIVE_INFINITY, minZ = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY, maxZ = Double.NEGATIVE_INFINITY;
        for (Vector3f vertex : obb.getVertices()) {
            minX = Math.min(minX, vertex.x); minY = Math.min(minY, vertex.y); minZ = Math.min(minZ, vertex.z);
            maxX = Math.max(maxX, vertex.x); maxY = Math.max(maxY, vertex.y); maxZ = Math.max(maxZ, vertex.z);
        }
        return new AABB(minX, minY, minZ, maxX, maxY, maxZ);
    }

    /**
     * Mirrors the native main-OBB contact rule when the AABB broad phase finds a block.
     * The vehicle's physics deliberately ignores low samples on the four vertical faces so
     * tracked hulls can climb/slide across curbs.  Treating those samples as hard walls made
     * the route planner reject passages that the very same vehicle could drive through.
     */
    private static TrackedHullPoint trackedPoseBlockingContact(ServerLevel level, TrackedPoseTransform transform) {
        for (TrackedHullPoint sample : transform.hull.samples) {
            Vec3 world = transform.world(sample.local);
            if (!level.getBlockState(BlockPos.containing(world)).isSolid()) continue;
            if (trackedPoseContactBlocksMotion(transform.hull, sample)) return sample;
        }
        return null;
    }

    private static boolean trackedPoseContactBlocksMotion(TrackedHull hull, TrackedHullPoint sample) {
        return switch (sample.face) {
            // This is the exact height condition from PhysicsEngine.motionByImpact().
            // Low side/front/back samples are track contact, not an impassable wall.
            case LEFT, RIGHT, FRONT, BACK -> sample.local.y >= hull.lowSideContactY;
            // Ground support is required separately; a bottom-face sample is not a blocker.
            case BOTTOM -> false;
            // A solid at the top sample is a real ceiling obstacle.
            case TOP -> true;
        };
    }

    private static int trackedPoseHeading(float yaw) {
        return Math.floorMod(Math.round(yaw / (360.0F / TRACKED_POSE_HEADINGS)), TRACKED_POSE_HEADINGS);
    }

    private static float trackedPoseYaw(int heading) {
        return Mth.wrapDegrees(heading * (360.0F / TRACKED_POSE_HEADINGS));
    }

    private static TrackedPoseKey trackedPoseKey(Vec3 anchor, Vec3 position, int heading) {
        return new TrackedPoseKey((int) Math.round((position.x - anchor.x) / TRACKED_POSE_CELL),
                (int) Math.round((position.z - anchor.z) / TRACKED_POSE_CELL), heading);
    }

    private static Vec3 trackedPosePosition(Vec3 anchor, TrackedPoseKey key, double y) {
        return new Vec3(anchor.x + key.x * TRACKED_POSE_CELL, y, anchor.z + key.z * TRACKED_POSE_CELL);
    }

    private static Vec3 navigationTarget(ServerPlayer player, Entity vehicle, Vec3 finalTarget, VehicleShape shape) {
        if (vehicle.getPersistentData().getBoolean(PATH_ASYNC_PENDING)) {
            // Route snapshots intentionally take several ticks.  Returning null here
            // makes move() call stopVehicle(), so a tracked vehicle near an open
            // house passage receives one steering tick and then waits for the whole
            // async search.  Keep an orientation target instead: move() will replay
            // an in-place pivot (or braking hold), never translation, until the result
            // is based on the same physical position.
            Vec3 trackedAdvance = trackedAsyncProgressTarget(vehicle, finalTarget, shape);
            if (trackedAdvance != null) return trackedAdvance;
            return null;
        }
        double speed = horizontalSpeed(vehicle);
        if (flatDistance(vehicle.position(), finalTarget) <= directFinalRange(shape)
                && canTravelDirect(vehicle, vehicle.position(), finalTarget, shape, directFinalRange(shape))) {
            return finalTarget;
        }
        CompoundTag data = vehicle.getPersistentData();
        if (data.getBoolean(PATH_BLOCKED)) {
            data.remove(PATH_BLOCKED);
            double lookahead = dynamicLookahead(speed, shape);
            Vec3 lookaheadTarget = clippedTarget(vehicle.position(), finalTarget, lookahead);
            if (canTravelDirect(vehicle, vehicle.position(), lookaheadTarget, shape, Math.max(PATH_LOOKAHEAD_MIN, flatDistance(vehicle.position(), lookaheadTarget) + shape.radius()))) {
                clearRoute(vehicle);
                pathDebug(vehicle, "BLOCKED_DIRECT_RECOVER", "lookahead=%s final=%s", fmt(lookaheadTarget), fmt(finalTarget));
                return lookaheadTarget;
            }
        }
        Vec3 routePoint = nextRoutePoint(vehicle, finalTarget, shape, speed);
        if (routePoint != null) return routePoint;
        if (!pathPolicy(vehicle).equals(PATH_POLICY_DIRECT)) {
            long now = vehicle.level().getGameTime();
            if (now >= data.getLong(REPLAN_AFTER)) {
                data.putLong(REPLAN_AFTER, now + PATH_REPLAN_COOLDOWN_TICKS);
                pathDebug(vehicle, "REPATH_START", "final=%s", fmt(finalTarget));
                prepareRoute(player, vehicle, finalTarget, Set.of(vehicle.getUUID()));
                routePoint = nextRoutePoint(vehicle, finalTarget, shape, speed);
                if (routePoint != null) return routePoint;
            }
        }
        double lookahead = dynamicLookahead(speed, shape);
        return clippedTarget(vehicle.position(), finalTarget, lookahead);
    }

    private static Vec3 trackedAsyncProgressTarget(Entity vehicle, Vec3 finalTarget, VehicleShape shape) {
        if (!isTrackedVehicle(vehicle) || finalTarget == null) return null;
        double distance = flatDistance(vehicle.position(), finalTarget);
        if (distance < 1.0E-6D) return null;
        // Keep the temporary orientation target close.  The pending-route branch in
        // move() consumes it only for yaw and prevents translation.
        double advance = Mth.clamp(shape.radius() * 2.0D, 6.0D, 10.0D);
        return clippedTarget(vehicle.position(), finalTarget, advance);
    }

    private static boolean startAsyncRoute(Entity vehicle, Vec3 safe, VehicleShape shape, Set<UUID> ignored, long generation) {
        if (ASYNC_ROUTES.containsKey(vehicle.getUUID())) return true;
        Vec3 start = vehicle.position();
        int radius = Math.min(32, (int) Math.ceil(PATH_SEARCH_RADIUS / PATH_STEP));
        int gx = Mth.clamp((int) Math.round((safe.x - start.x) / PATH_STEP), -radius, radius);
        int gz = Mth.clamp((int) Math.round((safe.z - start.z) / PATH_STEP), -radius, radius);
        ASYNC_ROUTES.put(vehicle.getUUID(), new AsyncRouteBuild(start, safe, shape, ignored == null ? Set.of() : Set.copyOf(ignored), generation, radius, gx, gz));
        return true;
    }

    private static void advanceAsyncRoute(ServerPlayer player, Entity vehicle, Vec3 target, Set<UUID> ignored) {
        AsyncRouteBuild build = ASYNC_ROUTES.get(vehicle.getUUID());
        CompoundTag data = vehicle.getPersistentData();
        if (build == null || build.generation != data.getLong(PATH_GENERATION)) { data.remove(PATH_ASYNC_PENDING); return; }
        if (build.future == null) {
            int budget = ASYNC_SNAPSHOT_CELLS_PER_TICK;
            while (budget-- > 0 && build.cursor < build.points.size()) {
                DominionAsyncGridPlanner.Point point = build.points.get(build.cursor++);
                Vec3 raw = build.start.add(point.x() * PATH_STEP, 0.0D, point.z() * PATH_STEP);
                Vec3 occupy = occupiableNear(vehicle, raw, build.shape, build.ignored);
                if (occupy != null) {
                    double penalty = terrainPenalty(vehicle, occupy) + otherVehiclePenalty(vehicle, occupy, build.shape, build.ignored) + reservationPenalty(vehicle, occupy, build.shape);
                    build.cells.put(point.key(), new DominionAsyncGridPlanner.Cell(occupy.y, penalty));
                }
            }
            if (build.cursor < build.points.size()) return;
            DominionAsyncGridPlanner.Point startPoint = new DominionAsyncGridPlanner.Point(0, 0);
            DominionAsyncGridPlanner.Point goalPoint = new DominionAsyncGridPlanner.Point(build.goalX, build.goalZ);
            build.future = DominionAsyncGridPlanner.submit(new DominionAsyncGridPlanner.Snapshot(startPoint, goalPoint, Map.copyOf(build.cells), PATH_MAX_ITERATIONS, terrainGridStepHeight(), 2.0D));
            return;
        }
        if (!build.future.isDone()) return;
        DominionAsyncGridPlanner.Result result = build.future.getNow(null);
        ASYNC_ROUTES.remove(vehicle.getUUID());
        data.remove(PATH_ASYNC_PENDING);
        if (result == null || !result.found()) { data.putBoolean(PATH_BLOCKED, true); return; }
        List<Vec3> route = new ArrayList<>();
        for (DominionAsyncGridPlanner.Point point : result.points()) {
            DominionAsyncGridPlanner.Cell cell = build.cells.get(point.key());
            if (cell != null) route.add(new Vec3(build.start.x + point.x() * PATH_STEP, cell.y(), build.start.z + point.z() * PATH_STEP));
        }
        if (route.size() <= 1 || !validateAsyncRoute(vehicle, route, build.shape, build.ignored)) { data.putBoolean(PATH_BLOCKED, true); return; }
        // The async solver deliberately works on a coarse, conservative grid.  Its raw
        // result is a chain of every grid cell, not a route a vehicle should literally
        // steer through.  Keeping that chain made a tracked hull follow needless large
        // U-turns around a house even when two later points had a clear swept segment.
        // Compress only by the same continuous collision sweep used at drive time, then
        // validate again with the fleet's ignored-vehicle set before committing it.
        int rawPoints = route.size();
        route = simplifyRoute(vehicle, route, build.shape);
        if (!validateAsyncRoute(vehicle, route, build.shape, build.ignored)) { data.putBoolean(PATH_BLOCKED, true); return; }
        if (flatDistance(route.get(route.size() - 1), build.safe) > 1.0D) route.add(build.safe);
        storeRoute(vehicle, build.safe, route, firstUsefulIndex(route, vehicle.position(), build.shape));
        refreshPlannedPath(player, vehicle, route);
        pathDebug(vehicle, "ASYNC_ROUTE_APPLIED", "generation=%d cells=%d visited=%d rawPoints=%d points=%d", build.generation, build.cells.size(), result.visited(), rawPoints, route.size());
    }

    private static boolean validateAsyncRoute(Entity vehicle, List<Vec3> route, VehicleShape shape, Set<UUID> ignored) {
        for (int i = 1; i < route.size(); i++) if (!canSweep(vehicle, route.get(i - 1), route.get(i), shape, ignored)) return false;
        return true;
    }

    private static Vec3 nextRoutePoint(Entity vehicle, Vec3 finalTarget, VehicleShape shape, double speed) {
        CompoundTag data = vehicle.getPersistentData();
        if (!data.contains(PATH_POINTS, Tag.TAG_LIST)) return null;
        ListTag points = data.getList(PATH_POINTS, Tag.TAG_COMPOUND);
        if (points.size() < 2) return null;
        int index = Mth.clamp(data.getInt(PATH_INDEX), 1, points.size() - 1);
        Vec3 position = vehicle.position();
        while (index < points.size()) {
            Vec3 point = readPathPoint(points.getCompound(index));
            Vec3 previous = index > 0 ? readPathPoint(points.getCompound(index - 1)) : null;
            Vec3 next = index + 1 < points.size() ? readPathPoint(points.getCompound(index + 1)) : null;
            if (point == null) {
                clearRoute(vehicle);
                return null;
            }
            double reach = Math.max(2.0D, shape.radius() * 0.55D);
            if (flatDistance(position, point) <= reach || hasPassedPoint(position, previous, point) || canSkipAligned(vehicle, position, previous, point, next, shape)) {
                index++;
                data.putInt(PATH_INDEX, index);
                continue;
            }
            data.putInt(PATH_INDEX, index);
            return routeLookaheadPoint(vehicle, points, index, position, finalTarget, shape, speed);
        }
        clearRoute(vehicle);
        return finalTarget;
    }

    private static Vec3 routeLookaheadPoint(Entity vehicle, ListTag points, int index, Vec3 position, Vec3 finalTarget, VehicleShape shape, double speed) {
        double lookahead = dynamicLookahead(speed, shape);
        Vec3 previous = position;
        Vec3 lastVisible = null;
        double walked = 0.0D;
        for (int i = Math.max(1, index); i < points.size(); i++) {
            Vec3 point = readPathPoint(points.getCompound(i));
            if (point == null) break;
            // A lookahead point is only a valid steering target when the vehicle can
            // reach it directly from its *current* position.  Previously this method
            // jumped several grid points ahead merely because their accumulated route
            // distance was within lookahead.  At a house corner that cuts across the
            // planned route, commands a broad pivot toward the far point, and drives
            // the hull into the building the grid had just avoided.
            double directDistance = Math.max(PATH_LOOKAHEAD_MIN, flatDistance(position, point) + shape.radius());
            if (!canTravelDirect(vehicle, position, point, shape, directDistance)) break;
            lastVisible = point;
            walked += flatDistance(previous, point);
            if (walked >= lookahead) return point;
            previous = point;
        }
        // Never substitute the final destination after visibility was lost: that is
        // exactly the same forbidden corner-cut at a much larger scale.  The nearest
        // visible point keeps the hull inside the route; if no route point is currently
        // visible, use the first point so ensureRoute can trigger its normal replan.
        if (lastVisible != null) return lastVisible;
        Vec3 first = index >= 0 && index < points.size() ? readPathPoint(points.getCompound(index)) : null;
        return first != null ? first : finalTarget;
    }

    private static double routeCurvatureSpeedLimit(Entity vehicle, Vec3 currentTarget, double speed) {
        CompoundTag data = vehicle.getPersistentData();
        if (!data.contains(PATH_POINTS, Tag.TAG_LIST)) return Double.POSITIVE_INFINITY;
        ListTag points = data.getList(PATH_POINTS, Tag.TAG_COMPOUND);
        int index = Mth.clamp(data.getInt(PATH_INDEX), 1, Math.max(1, points.size() - 1));
        if (points.size() < 3 || index >= points.size() - 1) return Double.POSITIVE_INFINITY;
        double maxAngle = 0.0D;
        Vec3 a = vehicle.position();
        Vec3 b = currentTarget;
        int lookahead = Mth.clamp(2 + (int) Math.floor(speed * 20.0D), 2, 6);
        for (int i = index + 1; i < points.size() && i <= index + lookahead; i++) {
            Vec3 c = readPathPoint(points.getCompound(i));
            if (b != null && c != null) maxAngle = Math.max(maxAngle, turnAngle(a, b, c));
            a = b;
            b = c;
        }
        if (maxAngle <= 15.0D) return Double.POSITIVE_INFINITY;
        if (maxAngle > 60.0D) return 0.12D;
        if (maxAngle > 35.0D) return 0.20D;
        if (maxAngle > 20.0D) return 0.30D;
        return 0.34D;
    }

    private static List<Vec3> findAvoidancePath(Entity vehicle, Vec3 start, Vec3 target, VehicleShape shape, Set<UUID> ignoredVehicles) {
        try (LagTrace ignoredTrace = LagTrace.start("ywzj.route.astar", "vehicle=" + vehicle.getId())) {
        Vec3 startPos = occupiableNear(vehicle, start, shape, ignoredVehicles);
        Vec3 targetPos = occupiableNear(vehicle, target, shape, ignoredVehicles);
        if (startPos == null || targetPos == null) {
            pathDebug(vehicle, "PATH_SEARCH_FAILED", "start=%s target=%s reason=%s", fmt(start), fmt(target), startPos == null ? "bad_start" : "bad_target");
            LagTrace.mark("astar_failed:bad_endpoint");
            return List.of();
        }
        AvoidNode startNode = new AvoidNode(0, 0);
        AvoidNode best = startNode;
        double bestH = flatDistance(startPos, targetPos);
        PriorityQueue<PathState> open = new PriorityQueue<>(Comparator.comparingDouble(PathState::f));
        Map<AvoidNode, Double> cost = new HashMap<>();
        Map<AvoidNode, AvoidNode> parent = new HashMap<>();
        Set<AvoidNode> closed = new HashSet<>();
        Map<AvoidNode, Vec3> occupancies = new HashMap<>();
        occupancies.put(startNode, startPos);
        open.add(new PathState(startNode, 0.0D, bestH));
        cost.put(startNode, 0.0D);
        int iterations = 0;
        while (!open.isEmpty() && iterations++ < PATH_MAX_ITERATIONS) {
            PathState currentState = open.poll();
            AvoidNode current = currentState.node();
            if (!closed.add(current)) continue;
            Vec3 currentWorld = worldPos(startPos, current);
            double h = flatDistance(currentWorld, targetPos);
            if (h < bestH) {
                bestH = h;
                best = current;
            }
            if (h <= Math.max(PATH_STEP, shape.radius() * 0.6D)) {
                best = current;
                break;
            }
            Vec3 currentOccupy = cachedOccupiableNear(vehicle, currentWorld, current, shape, ignoredVehicles, occupancies);
            if (currentOccupy == null) continue;
            for (int dx = -1; dx <= 1; dx++) for (int dz = -1; dz <= 1; dz++) {
                if (dx == 0 && dz == 0) continue;
                AvoidNode next = new AvoidNode(current.x() + dx, current.z() + dz);
                if (closed.contains(next)) continue;
                Vec3 world = worldPos(startPos, next);
                if (flatDistance(startPos, world) > PATH_SEARCH_RADIUS) continue;
                Vec3 occupy = cachedOccupiableNear(vehicle, world, next, shape, ignoredVehicles, occupancies);
                if (occupy == null || !canSweep(vehicle, currentOccupy, occupy, shape, ignoredVehicles)) continue;
                double step = (dx != 0 && dz != 0 ? PATH_STEP * 1.414D : PATH_STEP);
                double penalty = terrainPenalty(vehicle, occupy) + otherVehiclePenalty(vehicle, occupy, shape, ignoredVehicles) + reservationPenalty(vehicle, occupy, shape);
                double nextCost = cost.getOrDefault(current, Double.POSITIVE_INFINITY) + step + penalty;
                if (nextCost >= cost.getOrDefault(next, Double.POSITIVE_INFINITY)) continue;
                cost.put(next, nextCost);
                parent.put(next, current);
                open.add(new PathState(next, nextCost, flatDistance(occupy, targetPos)));
            }
        }
        double acceptableBest = Math.max(PATH_STEP * 2.0D, shape.radius() + PATH_STEP);
        if (bestH > acceptableBest) {
            pathDebug(vehicle, "PATH_SEARCH_FAILED", "start=%s target=%s bestDistance=%.2f iterations=%d reason=no_reachable_node", fmt(start), fmt(target), bestH, iterations);
            LagTrace.mark("astar_failed:iterations=" + iterations);
            return List.of();
        }
        List<AvoidNode> nodes = new ArrayList<>();
        for (AvoidNode cursor = best; cursor != null; cursor = parent.get(cursor)) nodes.add(cursor);
        Collections.reverse(nodes);
        List<Vec3> route = new ArrayList<>();
        for (AvoidNode node : nodes) {
            Vec3 occupy = cachedOccupiableNear(vehicle, worldPos(startPos, node), node, shape, ignoredVehicles, occupancies);
            if (occupy != null) route.add(occupy);
        }
        if (route.isEmpty()) route.add(startPos);
        Vec3 last = route.get(route.size() - 1);
        if (flatDistance(last, targetPos) > 1.0D && canSweep(vehicle, last, targetPos, shape, ignoredVehicles)) route.add(targetPos);
        LagTrace.mark("astar_route:iterations=" + iterations + ":points=" + route.size());
        return route;
        }
    }

    private static List<Vec3> simplifyRoute(Entity vehicle, List<Vec3> raw, VehicleShape shape) {
        if (raw.size() <= 2) return raw;
        List<Vec3> result = new ArrayList<>();
        int i = 0;
        result.add(raw.get(0));
        while (i < raw.size() - 1) {
            int best = i + 1;
            Vec3 from = raw.get(i);
            for (int j = i + 2; j < raw.size(); j++) {
                Vec3 to = raw.get(j);
                if (!canTravelDirect(vehicle, from, to, shape, flatDistance(from, to) + shape.radius())) break;
                best = j;
            }
            result.add(raw.get(best));
            i = best;
        }
        return result;
    }

    private static boolean canTravelDirect(Entity vehicle, Vec3 from, Vec3 to, VehicleShape shape, double maxDistance) {
        double distance = flatDistance(from, to);
        if (distance < 1.0E-6D) return true;
        Vec3 dir = to.subtract(from).multiply(1.0D, 0.0D, 1.0D).normalize();
        double checked = Math.min(distance, maxDistance);
        Vec3 previous = occupiableNear(vehicle, from, shape, Set.of(vehicle.getUUID()));
        if (previous == null) previous = from;
        for (double d = 0.0D; d <= checked + 0.01D; d += 1.0D) {
            Vec3 sample = terrainProbe(from.add(dir.scale(Math.min(d, checked))), previous.y);
            Vec3 occupy = occupiableForTravel(vehicle, sample, shape, Set.of(vehicle.getUUID()), previous.y);
            if (occupy == null) return false;
            previous = occupy;
        }
        return true;
    }

    private static boolean canSweep(Entity vehicle, Vec3 from, Vec3 to, VehicleShape shape, Set<UUID> ignoredVehicles) {
        double distance = flatDistance(from, to);
        if (distance < 1.0E-6D) return true;
        Vec3 dir = to.subtract(from).multiply(1.0D, 0.0D, 1.0D).normalize();
        Vec3 previous = occupiableNear(vehicle, from, shape, ignoredVehicles);
        if (previous == null) previous = from;
        for (double d = 1.0D; d <= distance + 0.01D; d += 1.0D) {
            Vec3 sample = terrainProbe(from.add(dir.scale(Math.min(d, distance))), previous.y);
            Vec3 occupy = occupiableForTravel(vehicle, sample, shape, ignoredVehicles, previous.y);
            if (occupy == null) return false;
            previous = occupy;
        }
        return true;
    }

    private static Vec3 safeTargetNear(Entity vehicle, Vec3 target, VehicleShape shape, Set<UUID> ignoredVehicles) {
        Vec3 direct = occupiableNear(vehicle, target, shape, ignoredVehicles);
        Vec3 origin = vehicle.position();
        if (direct != null && canTravelDirect(vehicle, origin, direct, shape,
                Math.max(PATH_LOOKAHEAD_MIN, flatDistance(origin, direct) + shape.radius()))) return direct;
        List<SafeCandidate> candidates = new ArrayList<>();
        for (double radius = PATH_STEP; radius <= Math.max(18.0D, shape.radius() * 3.0D); radius += PATH_STEP) {
            for (int angle = 0; angle < 360; angle += 20) {
                double radians = Math.toRadians(angle);
                Vec3 candidate = target.add(Math.cos(radians) * radius, 0.0D, Math.sin(radians) * radius);
                Vec3 occupy = occupiableNear(vehicle, candidate, shape, ignoredVehicles);
                if (occupy == null) continue;
                double score = flatDistance(occupy, target) + flatDistance(origin, occupy) * 0.05D;
                candidates.add(new SafeCandidate(occupy, score));
            }
            if (!candidates.isEmpty()) break;
        }
        if (candidates.isEmpty()) return target;
        candidates.sort(Comparator.comparingDouble(SafeCandidate::score));
        for (SafeCandidate candidate : candidates) {
            if (canTravelDirect(vehicle, origin, candidate.position(), shape, Math.max(PATH_LOOKAHEAD_MIN, flatDistance(origin, candidate.position()) + shape.radius()))) {
                return candidate.position();
            }
        }
        // The planner may still route around a real obstacle, but never claim that a
        // disconnected roof/platform is an immediately reachable destination.
        return candidates.get(0).position();
    }

    private static Vec3 occupiableForTravel(Entity vehicle, Vec3 around, VehicleShape shape, Set<UUID> ignoredVehicles, double referenceY) {
        Vec3 occupy = occupiableNear(vehicle, terrainProbe(around, referenceY), shape, ignoredVehicles);
        if (occupy == null) return null;
        if (!terrainStepAllowed(referenceY, occupy.y)) return null;
        return occupy;
    }

    private static Vec3 occupiableNear(Entity vehicle, Vec3 around, VehicleShape shape, Set<UUID> ignoredVehicles) {
        if (!(vehicle.level() instanceof ServerLevel level)) return around;
        Vec3 local = occupiableNearHeight(vehicle, around, shape, ignoredVehicles, around.y);
        if (local != null) return local;

        // The route grid has X/Z nodes only.  Reconstructing a node from its origin
        // must not make a long downhill disappear after the local probe window is
        // exhausted.  This is only a terrain candidate; every edge is still swept
        // through the real collision space before it becomes a route segment.
        double terrainY = level.getHeight(Heightmap.Types.MOTION_BLOCKING_NO_LEAVES,
                Mth.floor(around.x), Mth.floor(around.z));
        return Math.abs(terrainY - around.y) < 1.0E-6D
                ? null
                : occupiableNearHeight(vehicle, around, shape, ignoredVehicles, terrainY);
    }

    private static Vec3 occupiableNearHeight(Entity vehicle, Vec3 around, VehicleShape shape, Set<UUID> ignoredVehicles, double baseY) {
        if (!(vehicle.level() instanceof ServerLevel level)) return null;
        BlockPos base = BlockPos.containing(around.x, around.y, around.z);
        if (base.getY() != Mth.floor(baseY)) base = BlockPos.containing(around.x, baseY, around.z);
        for (int dy = 2; dy >= -4; dy--) {
            BlockPos floor = base.offset(0, dy - 1, 0);
            if (level.getBlockState(floor).getCollisionShape(level, floor).isEmpty()) continue;
            Vec3 candidate = new Vec3(around.x, floor.getY() + 1.0D, around.z);
            if (canOccupy(vehicle, candidate, shape, ignoredVehicles)) return candidate;
        }
        return null;
    }

    private static Vec3 cachedOccupiableNear(Entity vehicle, Vec3 around, AvoidNode node, VehicleShape shape,
                                               Set<UUID> ignoredVehicles, Map<AvoidNode, Vec3> cache) {
        if (cache.containsKey(node)) return cache.get(node);
        Vec3 occupy = occupiableNear(vehicle, around, shape, ignoredVehicles);
        cache.put(node, occupy);
        return occupy;
    }

    /** Uses the last accepted floor height so a route follows terrain instead of its starting plane. */
    static Vec3 terrainProbe(Vec3 horizontalSample, double referenceY) {
        return new Vec3(horizontalSample.x, referenceY, horizontalSample.z);
    }

    /** A continuous natural slope is valid; a wall-sized vertical jump is not. */
    static boolean terrainStepAllowed(double previousY, double nextY) {
        return Double.isFinite(previousY) && Double.isFinite(nextY)
                && Math.abs(nextY - previousY) <= MAX_TRAVEL_STEP_HEIGHT;
    }

    /** The asynchronous grid samples three horizontal blocks at a time. */
    static double terrainGridStepHeight() {
        return terrainGridStepHeight(PATH_STEP);
    }

    static double terrainGridStepHeight(double gridStep) {
        return gridStep * MAX_TRAVEL_STEP_HEIGHT;
    }

    private static boolean canOccupy(Entity vehicle, Vec3 position, VehicleShape shape, Set<UUID> ignoredVehicles) {
        if (!canOccupyVoxelVehicleSpace(vehicle, position, shape)) return false;
        AABB box = shape.aabbAt(position);
        for (Entity other : vehicle.level().getEntities(vehicle, box.inflate(0.05D))) {
            if (other == vehicle || !isInstance(other, VEHICLE_CLASS_NAME)) continue;
            if (ignoredVehicles != null && ignoredVehicles.contains(other.getUUID())) continue;
            if (other.getBoundingBox().intersects(box.inflate(0.15D))) return false;
        }
        return true;
    }

    private static boolean canOccupyVoxelVehicleSpace(Entity vehicle, Vec3 position, VehicleShape shape) {
        if (!(vehicle.level() instanceof ServerLevel level)) return true;
        // WheeledVehicle uses a rigid native collision chassis.  It was working with
        // this strict volume check before 1.2.49; applying the tracked-vehicle
        // suspension profile to it made the planner approve positions its chassis
        // could not actually drive through.  Keep its proven pre-1.2.49 behavior.
        if (!isTrackedVehicle(vehicle)) return canOccupyStrictVehicleSpace(level, vehicle, position, shape);

        int height = Math.max(2, shape.voxelHeight());
        BlockPos center = BlockPos.containing(position.x, position.y + shape.voxelYOffset(), position.z);
        // A vehicle's native physics follows its suspension and can span a natural
        // slope. A single horizontal AABB cannot: on a long 1:1 descent, terrain at
        // the uphill side of a wide hull sits several blocks higher than its center.
        // Keep clearance at the route center, then validate the terrain *profile*
        // beneath the footprint. This rejects cliffs and walls, not a continuous slope.
        BlockPos floor = center.below();
        if (level.getBlockState(floor).getCollisionShape(level, floor).isEmpty()) return false;
        for (int dy = 0; dy < height; dy++) {
            BlockPos pos = center.above(dy);
            if (!level.getBlockState(pos).getCollisionShape(level, pos).isEmpty()) return false;
        }
        // Use the current physical OBB's world-space envelope, not a square made
        // from its longest edge.  A tracked chassis can pivot before entering a
        // turn, so its route probe may keep the current real footprint.
        AABB footprint = shape.aabbAt(position);
        int minX = Mth.floor(footprint.minX + 1.0E-4D);
        int maxX = Mth.floor(footprint.maxX - 1.0E-4D);
        int minZ = Mth.floor(footprint.minZ + 1.0E-4D);
        int maxZ = Mth.floor(footprint.maxZ - 1.0E-4D);
        double[][] terrain = new double[Math.max(1, maxX - minX + 1)][Math.max(1, maxZ - minZ + 1)];
        for (int x = 0; x < terrain.length; x++) {
            for (int z = 0; z < terrain[x].length; z++) {
                terrain[x][z] = level.getHeight(Heightmap.Types.MOTION_BLOCKING_NO_LEAVES,
                        minX + x, minZ + z);
            }
        }
        return terrainFootprintProfileAllowed(terrain);
    }

    /** Original full-hull collision test used by rigid wheeled vehicles. */
    private static boolean canOccupyStrictVehicleSpace(ServerLevel level, Entity vehicle, Vec3 position, VehicleShape shape) {
        // The old test inflated a wheeled car into a square based on every decorative OBB.
        // Query the actual Entity collision box instead: it is the footprint native driving
        // can genuinely pass through.
        AABB body = shape.aabbAt(position);
        if (!level.noCollision(vehicle, body)) return false;
        double insetX = Math.min(0.12D, Math.max(0.0D, body.getXsize() * 0.20D));
        double insetZ = Math.min(0.12D, Math.max(0.0D, body.getZsize() * 0.20D));
        AABB supportProbe = new AABB(
                body.minX + insetX, body.minY - 0.18D, body.minZ + insetZ,
                body.maxX - insetX, body.minY - 0.02D, body.maxZ - insetZ);
        return !level.noCollision(vehicle, supportProbe);
    }

    /** Adjacent terrain samples may differ by one natural step, but never by a wall-sized ledge. */
    static boolean terrainFootprintProfileAllowed(double[][] terrain) {
        if (terrain == null || terrain.length == 0) return false;
        for (int x = 0; x < terrain.length; x++) {
            if (terrain[x] == null || terrain[x].length == 0) return false;
            for (int z = 0; z < terrain[x].length; z++) {
                double current = terrain[x][z];
                if (!Double.isFinite(current)) return false;
                if (x > 0 && (z >= terrain[x - 1].length || !terrainStepAllowed(current, terrain[x - 1][z]))) return false;
                if (z > 0 && !terrainStepAllowed(current, terrain[x][z - 1])) return false;
            }
        }
        return true;
    }

    private static double otherVehiclePenalty(Entity vehicle, Vec3 position, VehicleShape shape, Set<UUID> ignoredVehicles) {
        AABB box = shape.aabbAt(position).inflate(shape.radius() * 0.65D, 0.5D, shape.radius() * 0.65D);
        double penalty = 0.0D;
        for (Entity other : vehicle.level().getEntities(vehicle, box)) {
            if (other == vehicle || !isInstance(other, VEHICLE_CLASS_NAME)) continue;
            if (ignoredVehicles != null && ignoredVehicles.contains(other.getUUID())) continue;
            double distance = Math.max(0.1D, flatDistance(position, other.position()));
            penalty += Math.max(0.0D, 18.0D - distance) * 1.5D;
        }
        return penalty;
    }

    private static void rememberFutureFootprint(Entity vehicle, Vec3 target, VehicleShape shape) {
        if (vehicle == null || target == null) return;
        long now = vehicle.level().getGameTime();
        RESERVATIONS.entrySet().removeIf(entry -> entry.getValue().expiresAt() < now || entry.getValue().owner().equals(vehicle.getUUID()));
        Vec3 start = vehicle.position();
        Vec3 delta = target.subtract(start).multiply(1.0D, 0.0D, 1.0D);
        double distance = Math.min(delta.length(), dynamicLookahead(horizontalSpeed(vehicle), shape));
        if (distance < 1.0E-6D) return;
        Vec3 dir = delta.normalize();
        double speed = horizontalSpeed(vehicle);
        for (double d = 0.0D; d <= distance + 0.01D; d += 1.5D) {
            Vec3 sample = start.add(dir.scale(Math.min(d, distance)));
            BlockPos center = BlockPos.containing(sample.x, sample.y, sample.z);
            int radius = Math.max(1, (int) Math.ceil(shape.radius()));
            for (int dx = -radius; dx <= radius; dx++) for (int dz = -radius; dz <= radius; dz++) {
                if (dx * dx + dz * dz > radius * radius) continue;
                RESERVATIONS.put(center.offset(dx, 0, dz).asLong(), new Reservation(vehicle.getUUID(), now + RESERVATION_TTL_TICKS, speed));
            }
        }
    }

    private static double reservationPenalty(Entity vehicle, Vec3 position, VehicleShape shape) {
        long now = vehicle.level().getGameTime();
        BlockPos center = BlockPos.containing(position.x, position.y, position.z);
        int radius = Math.max(1, (int) Math.ceil(shape.radius()));
        double penalty = 0.0D;
        for (int dx = -radius; dx <= radius; dx++) for (int dz = -radius; dz <= radius; dz++) {
            Reservation reservation = RESERVATIONS.get(center.offset(dx, 0, dz).asLong());
            if (reservation == null || reservation.expiresAt() < now || reservation.owner().equals(vehicle.getUUID())) continue;
            penalty += reservation.ownerSpeed() < 0.05D ? 12.0D : 3.5D;
        }
        return penalty;
    }

    private static boolean shouldYield(Entity vehicle, Vec3 target, VehicleShape shape) {
        double penalty = reservationPenalty(vehicle, clippedTarget(vehicle.position(), target, Math.max(6.0D, shape.radius() * 1.5D)), shape);
        if (penalty <= 0.0D) return false;
        boolean bypass = lateralBypassAvailable(vehicle, target, shape);
        if (bypass) return false;
        pathDebug(vehicle, "YIELD_RESERVATION", "target=%s penalty=%.1f", fmt(target), penalty);
        return true;
    }

    private static boolean lateralBypassAvailable(Entity vehicle, Vec3 target, VehicleShape shape) {
        Vec3 forward = target.subtract(vehicle.position()).multiply(1.0D, 0.0D, 1.0D);
        if (forward.lengthSqr() < 1.0E-6D) return false;
        forward = forward.normalize();
        Vec3 right = new Vec3(-forward.z, 0.0D, forward.x);
        for (int side : new int[]{1, -1}) {
            Vec3 lane = vehicle.position().add(forward.scale(Math.max(8.0D, shape.radius() * 2.0D))).add(right.scale(side * Math.max(4.0D, shape.radius() * 1.2D)));
            if (occupiableNear(vehicle, lane, shape, Set.of(vehicle.getUUID())) != null
                    && canTravelDirect(vehicle, vehicle.position(), lane, shape, flatDistance(vehicle.position(), lane) + shape.radius())
                    && reservationPenalty(vehicle, lane, shape) <= 2.0D) return true;
        }
        return false;
    }

    private static double terrainPenalty(Entity vehicle, Vec3 position) {
        BlockPos below = BlockPos.containing(position.x, position.y - 0.1D, position.z);
        Block block = vehicle.level().getBlockState(below).getBlock();
        return TERRAIN_PENALTY_CACHE.computeIfAbsent(block, YwzjVehicleAdapter::terrainPenaltyForBlock);
    }

    private static double terrainPenaltyForBlock(Block block) {
        String path = block.builtInRegistryHolder().key().location().getPath().toLowerCase(Locale.ROOT);
        if (path.contains("water") || path.contains("lava")) return 30.0D;
        if (path.contains("leaves") || path.contains("snow")) return 3.0D;
        return 0.0D;
    }

    private static boolean hasActiveRoute(Entity vehicle, Vec3 target) {
        CompoundTag data = vehicle.getPersistentData();
        if (!data.contains(FINAL_TARGET_X) || !data.contains(FINAL_TARGET_Z)) return false;
        double dx = target.x - data.getDouble(FINAL_TARGET_X);
        double dz = target.z - data.getDouble(FINAL_TARGET_Z);
        return dx * dx + dz * dz <= 4.0D;
    }

    private static Vec3 activeSafeTarget(Entity vehicle, Vec3 fallback) {
        CompoundTag data = vehicle.getPersistentData();
        if (data.contains(SAFE_TARGET_X) && data.contains(SAFE_TARGET_Y) && data.contains(SAFE_TARGET_Z)) {
            return new Vec3(data.getDouble(SAFE_TARGET_X), data.getDouble(SAFE_TARGET_Y), data.getDouble(SAFE_TARGET_Z));
        }
        return fallback;
    }

    private static List<Vec3> storedRoute(Entity vehicle, Vec3 fallbackTarget) {
        CompoundTag data = vehicle.getPersistentData();
        if (!hasActiveRoute(vehicle, fallbackTarget) || !data.contains(PATH_POINTS, Tag.TAG_LIST)) return List.of(vehicle.position(), fallbackTarget);
        ListTag points = data.getList(PATH_POINTS, Tag.TAG_COMPOUND);
        List<Vec3> route = new ArrayList<>();
        for (int i = 0; i < points.size(); i++) {
            Vec3 point = readPathPoint(points.getCompound(i));
            if (point != null) route.add(point);
        }
        return route;
    }

    private static void storeRoute(Entity vehicle, Vec3 safeTarget, List<Vec3> route, int startIndex) {
        CompoundTag data = vehicle.getPersistentData();
        ListTag points = new ListTag();
        for (Vec3 point : route) {
            CompoundTag item = new CompoundTag();
            item.putDouble("x", point.x);
            item.putDouble("y", point.y);
            item.putDouble("z", point.z);
            points.add(item);
        }
        data.put(PATH_POINTS, points);
        data.putDouble(PATH_TARGET_X, safeTarget.x);
        data.putDouble(PATH_TARGET_Z, safeTarget.z);
        data.putInt(PATH_INDEX, Mth.clamp(startIndex, 1, Math.max(1, route.size() - 1)));
    }

    private static void clearRoute(Entity vehicle) {
        CompoundTag data = vehicle.getPersistentData();
        TRACKED_POSE_ROUTES.remove(vehicle.getUUID());
        TRACKED_POSE_BUILDS.remove(vehicle.getUUID());
        TRACKED_RECOVERIES.remove(vehicle.getUUID());
        TRACKED_MOTION_WATCHES.remove(vehicle.getUUID());
        data.remove(PATH_POINTS);
        data.remove(PATH_INDEX);
        data.remove(PATH_TARGET_X);
        data.remove(PATH_TARGET_Z);
    }

    private static Vec3 readPathPoint(CompoundTag item) {
        if (!item.contains("x") || !item.contains("y") || !item.contains("z")) return null;
        return new Vec3(item.getDouble("x"), item.getDouble("y"), item.getDouble("z"));
    }

    private static int firstUsefulIndex(List<Vec3> route, Vec3 position, VehicleShape shape) {
        double reach = Math.max(2.0D, shape.radius() * 0.55D);
        for (int i = 1; i < route.size(); i++) if (flatDistance(position, route.get(i)) > reach) return i;
        return Math.max(1, route.size() - 1);
    }

    private static boolean hasPassedPoint(Vec3 position, Vec3 previous, Vec3 point) {
        if (previous == null || point == null) return false;
        Vec3 segment = point.subtract(previous).multiply(1.0D, 0.0D, 1.0D);
        Vec3 beyond = position.subtract(point).multiply(1.0D, 0.0D, 1.0D);
        return segment.lengthSqr() > 1.0E-6D && beyond.dot(segment) > 0.0D;
    }

    private static boolean canSkipAligned(Entity vehicle, Vec3 position, Vec3 previous, Vec3 point, Vec3 next, VehicleShape shape) {
        if (point == null || next == null) return false;
        double angle = turnAngle(previous == null ? position : previous, point, next);
        if (angle > 28.0D) return false;
        return flatDistance(position, point) <= dynamicLookahead(horizontalSpeed(vehicle), shape)
                && canTravelDirect(vehicle, position, next, shape, flatDistance(position, next) + shape.radius());
    }

    private static Vec3 clippedTarget(Vec3 position, Vec3 target, double range) {
        Vec3 delta = target.subtract(position).multiply(1.0D, 0.0D, 1.0D);
        double distance = Math.sqrt(delta.lengthSqr());
        if (distance <= range || distance < 1.0E-6D) return target;
        return position.add(delta.normalize().scale(range));
    }

    private static double dynamicLookahead(double speed, VehicleShape shape) {
        return Mth.clamp(PATH_LOOKAHEAD_MIN + Math.max(0.0D, speed) * 22.0D + shape.radius() * 0.4D, PATH_LOOKAHEAD_MIN, PATH_LOOKAHEAD_MAX);
    }

    private static double directFinalRange(VehicleShape shape) {
        return Mth.clamp(shape.radius() * 3.0D + 8.0D, 15.0D, 28.0D);
    }

    private static double flatDistance(Vec3 a, Vec3 b) {
        double dx = a.x - b.x;
        double dz = a.z - b.z;
        return Math.sqrt(dx * dx + dz * dz);
    }

    private static double turnAngle(Vec3 a, Vec3 b, Vec3 c) {
        if (a == null || b == null || c == null) return 0.0D;
        Vec3 ab = b.subtract(a).multiply(1.0D, 0.0D, 1.0D);
        Vec3 bc = c.subtract(b).multiply(1.0D, 0.0D, 1.0D);
        if (ab.lengthSqr() < 1.0E-6D || bc.lengthSqr() < 1.0E-6D) return 0.0D;
        return Math.toDegrees(Math.acos(Mth.clamp(ab.normalize().dot(bc.normalize()), -1.0D, 1.0D)));
    }

    private static Vec3 worldPos(Vec3 origin, AvoidNode node) {
        return origin.add(node.x() * PATH_STEP, 0.0D, node.z() * PATH_STEP);
    }

    private static String pathPolicy(Entity vehicle) {
        String policy = vehicle.getPersistentData().getString(PATH_POLICY_NBT);
        if (PATH_POLICY_ALWAYS.equals(policy)) return PATH_POLICY_ALWAYS;
        if (PATH_POLICY_DIRECT.equals(policy)) return PATH_POLICY_DIRECT;
        return "auto_on_stuck";
    }

    private static Set<UUID> vehicleIdSet(List<Entity> vehicles) {
        Set<UUID> ids = new HashSet<>();
        if (vehicles != null) for (Entity vehicle : vehicles) if (vehicle != null && isInstance(vehicle, VEHICLE_CLASS_NAME)) ids.add(vehicle.getUUID());
        return ids;
    }

    private static void refreshPlannedPath(ServerPlayer player, Entity vehicle, List<Vec3> route) {
        if (route == null || route.size() < 2) return;
        ServerPlayer targetPlayer = player;
        if (targetPlayer == null && vehicle.level() instanceof ServerLevel level) {
            UUID controller = DominionControlApi.controller(vehicle);
            if (controller != null) targetPlayer = level.getServer().getPlayerList().getPlayer(controller);
        }
        if (targetPlayer != null) DominionControlApi.refreshPlannedVehiclePath(targetPlayer, vehicle, route);
    }

    private static boolean fireAllWeapons(Entity vehicle, LivingEntity target) {
        if (!(vehicle instanceof AbstractVehicle ywzjVehicle)) return false;
        Set<WeaponUnit> occupiedStations = Collections.newSetFromMap(new java.util.IdentityHashMap<>());
        for (AbstractVehicle.Seat seat : ywzjVehicle.seats) {
            if (seat.passengerId == null || seat.passengerId < 0) continue;
            Entity passenger = ywzjVehicle.level().getEntity(seat.passengerId);
            if (seat.partUnit instanceof WeaponUnit station) {
                WeaponUnit root = station.getRootParentWeaponUnit();
                if (root.getOwner() != null) occupiedStations.add(root);
                // A weapon operator is controlled through the vehicle weapon API.
                // Do not let the same Mob's handheld AI conceal or duplicate it.
                if (passenger instanceof Mob mob) {
                    mob.lookAt(target, 180.0F, 180.0F);
                    mob.getLookControl().setLookAt(target, 180.0F, 180.0F);
                    mob.setTarget(null);
                }
            } else if (passenger instanceof Mob mob) {
                // Ordinary passenger seats have no vehicle weapon, so retain their
                // native handheld attack against the vehicle command target.
                mob.setTarget(target);
                mob.lookAt(target, 180.0F, 180.0F);
                mob.getLookControl().setLookAt(target, 180.0F, 180.0F);
            }
        }
        boolean any = false;
        for (WeaponUnit station : occupiedStations) {
            any |= fireWeaponStation(ywzjVehicle, station, target);
        }
        return any;
    }

    private static boolean hasFixedSelectedWeapon(AbstractVehicle vehicle) {
        Set<WeaponUnit> checked = Collections.newSetFromMap(new java.util.IdentityHashMap<>());
        for (AbstractVehicle.Seat seat : vehicle.seats) {
            if (seat.passengerId == null || seat.passengerId < 0 || !(seat.partUnit instanceof WeaponUnit station)) continue;
            WeaponUnit root = station.getRootParentWeaponUnit();
            if (!checked.add(root)) continue;
            Optional<AbstractVehicleWeapon<?>> selected = root.getCurrentWeapon();
            if (selected.isEmpty()) continue;
            if (!isOffensiveWeapon(selected.get())) continue;
            WeaponUnit muzzle = selected.get().getWeaponUnit();
            if (muzzle.isParentWeaponUnitAim()) continue;
            double xRange = Math.abs(readFloat(readMember(muzzle, "xRotMax"), 0.0F)
                    - readFloat(readMember(muzzle, "xRotMin"), 0.0F));
            double yRange = Math.abs(readFloat(readMember(muzzle, "yRotMax"), 0.0F)
                    - readFloat(readMember(muzzle, "yRotMin"), 0.0F));
            if (xRange < 0.5D && yRange < 0.5D) return true;
        }
        return false;
    }

    private static boolean fireWeaponStation(AbstractVehicle vehicle, WeaponUnit root, LivingEntity target) {
        LivingEntity operator = root.getOwner();
        if (operator == null || !operator.isAlive()) return false;
        Vec3 targetCenter = target.getBoundingBox().getCenter();
        root.setLockedEntity(target);
        root.aim(targetCenter);
        Optional<AbstractVehicleWeapon<?>> primary = root.getCurrentWeapon();
        Optional<AbstractVehicleWeapon<?>> secondary = root.getCurrentSecondaryWeapon();
        if (primary.isEmpty() && secondary.isEmpty()) {
            traceHelicopterWeapon(operator, vehicle, root, null, target, "NO_SELECTED_WEAPON", Double.NaN);
            return false;
        }
        boolean any = false;
        if (primary.isPresent()) any |= fireSelectedWeapon(vehicle, root, primary.get(), operator, target, targetCenter);
        if (secondary.isPresent() && (primary.isEmpty() || secondary.get() != primary.get())) {
            any |= fireSelectedWeapon(vehicle, root, secondary.get(), operator, target, targetCenter);
        }
        return any;
    }

    private static boolean fireSelectedWeapon(AbstractVehicle vehicle, WeaponUnit root,
                                              AbstractVehicleWeapon<?> weapon, LivingEntity operator,
                                              LivingEntity target, Vec3 targetCenter) {
        if (!isOffensiveWeapon(weapon)) {
            traceHelicopterWeapon(operator, vehicle, root, weapon, target, "NON_OFFENSIVE", Double.NaN);
            return false;
        }
        WeaponUnit muzzle = weapon.getWeaponUnit();
        muzzle.setLockedEntity(target);
        muzzle.aim(targetCenter);
        if (!isAimSettled(root) || !isAimSettled(muzzle)) {
            traceHelicopterWeapon(operator, vehicle, root, weapon, target, "AIMING", Double.NaN);
            return false;
        }
        if (weapon.isReloading()) {
            traceHelicopterWeapon(operator, vehicle, root, weapon, target, "RELOADING", Double.NaN);
            return false;
        }
        if (weapon.isCoolingDown()) return false;
        if (!weapon.hasAmmo()) {
            if (weapon.canReload()) weapon.startReload();
            traceHelicopterWeapon(operator, vehicle, root, weapon, target, "NO_AMMO", Double.NaN);
            return false;
        }
        List<AimContext> aimContexts = buildAimContexts(muzzle, targetCenter, vehicle.getDeltaMovement());
        if (aimContexts.isEmpty()) {
            traceHelicopterWeapon(operator, vehicle, root, weapon, target, "NO_MUZZLE", Double.NaN);
            return false;
        }
        double aimError = maximumAimErrorDegrees(aimContexts, targetCenter);
        if (!Double.isFinite(aimError) || aimError > WEAPON_FIRE_ARC_DEGREES) {
            traceHelicopterWeapon(operator, vehicle, root, weapon, target, "OUT_OF_ARC", aimError);
            return false;
        }
        if (!hasClearMuzzleShot(vehicle, aimContexts, targetCenter)) {
            traceHelicopterWeapon(operator, vehicle, root, weapon, target, "BLOCKED", aimError);
            return false;
        }
        vehicle.shoot(root.getIndex(), weapon.getIndex(), aimContexts, operator);
        traceHelicopterWeapon(operator, vehicle, root, weapon, target, "FIRE", aimError);
        return true;
    }

    private static boolean isOffensiveWeapon(AbstractVehicleWeapon<?> weapon) {
        String name = weapon.getClass().getSimpleName().toLowerCase(Locale.ROOT);
        return !name.contains("decoy") && !name.contains("flare") && !name.contains("smoke");
    }

    private static List<AimContext> buildAimContexts(WeaponUnit weaponUnit, Vec3 targetCenter, Vec3 vehicleVelocity) {
        Object firingMode = weaponUnit.getFiringMode();
        String mode = firingMode == null ? "" : firingMode.toString().toUpperCase(Locale.ROOT);
        List<AimContext> contexts;
        if (mode.contains("SALVO")) {
            contexts = new ArrayList<>(weaponUnit.aimContexts());
        } else if (mode.contains("RIPPLE")) {
            AimContext single = weaponUnit.aimContext();
            contexts = single == null ? List.of() : new ArrayList<>(List.of(single));
        } else {
            return List.of();
        }
        for (AimContext context : contexts) {
            context.from = context.from.add(vehicleVelocity);
            context.position = targetCenter;
        }
        return contexts;
    }

    private static double maximumAimErrorDegrees(List<AimContext> contexts, Vec3 targetCenter) {
        double max = 0.0D;
        for (AimContext context : contexts) {
            Vec3 toTarget = targetCenter.subtract(context.from);
            if (toTarget.lengthSqr() < 1.0E-8D) continue;
            Vec3 shot = Vec3.directionFromRotation(context.direction.x, context.direction.y);
            double dot = Mth.clamp(shot.normalize().dot(toTarget.normalize()), -1.0D, 1.0D);
            max = Math.max(max, Math.toDegrees(Math.acos(dot)));
        }
        return max;
    }

    private static boolean hasClearMuzzleShot(Entity vehicle, List<AimContext> contexts, Vec3 targetCenter) {
        for (AimContext context : contexts) {
            HitResult hit = vehicle.level().clip(new ClipContext(context.from, targetCenter,
                    ClipContext.Block.COLLIDER, ClipContext.Fluid.NONE, vehicle));
            if (hit.getType() != HitResult.Type.MISS && hit.getLocation().distanceToSqr(targetCenter) > 2.25D) return false;
        }
        return true;
    }

    private static void traceHelicopterWeapon(LivingEntity operator, AbstractVehicle vehicle, WeaponUnit root,
                                              AbstractVehicleWeapon<?> weapon, LivingEntity target,
                                              String state, double aimError) {
        if (!(operator instanceof Mob mob) || !YwzjVehicleCompatConfig.flightControlTraceEnabled()) return;
        long now = vehicle.level().getGameTime();
        CompoundTag data = mob.getPersistentData();
        if (!"FIRE".equals(state)
                && data.getLong(HELI_LAST_WEAPON_TRACE_TICK)
                + YwzjVehicleCompatConfig.flightControlTraceIntervalTicks() > now) return;
        data.putLong(HELI_LAST_WEAPON_TRACE_TICK, now);
        LOGGER.info("[DS-YWZJ-HELI-WEAPON] tick={} vehicle={} operator={} station={} weapon={} target={} state={} aimError={} ammo={} cooldown={} reload={}",
                now, vehicle.getId(), operator.getId(), root.getIndex(),
                weapon == null ? "none" : weapon.getClass().getSimpleName(), target.getId(), state,
                decimal(aimError), weapon == null ? -1 : weapon.getRemainAmmo(),
                weapon != null && weapon.isCoolingDown(), weapon != null && weapon.isReloading());
    }

    private static boolean isAimSettled(Object weaponUnit) {
        float xRot = readFloat(readMember(weaponUnit, "xRot"), Float.NaN);
        float yRot = readFloat(readMember(weaponUnit, "yRot"), Float.NaN);
        float xAimRot = readFloat(readMember(weaponUnit, "xAimRot"), Float.NaN);
        float yAimRot = readFloat(readMember(weaponUnit, "yAimRot"), Float.NaN);
        if (!Float.isFinite(xRot) || !Float.isFinite(yRot) || !Float.isFinite(xAimRot) || !Float.isFinite(yAimRot)) return true;
        return Math.abs(Mth.wrapDegrees(xAimRot - xRot)) <= AIM_TOLERANCE
                && Math.abs(Mth.wrapDegrees(yAimRot - yRot)) <= AIM_TOLERANCE;
    }

    private static boolean hasClearShot(Entity vehicle, LivingEntity target) {
        Vec3 from = vehicle.position().add(0.0D, Math.max(1.0D, vehicle.getBbHeight() * 0.7D), 0.0D);
        Vec3 to = target.getBoundingBox().getCenter();
        HitResult hit = vehicle.level().clip(new ClipContext(from, to, ClipContext.Block.COLLIDER, ClipContext.Fluid.NONE, vehicle));
        return hit.getType() == HitResult.Type.MISS || hit.getLocation().distanceToSqr(to) <= 2.25D;
    }

    private static boolean shouldBrake(Entity vehicle, double distance, double speed, double absYaw) {
        double turnPenalty = absYaw >= TURN_IN_PLACE_ANGLE ? 0.75D : 0.0D;
        double deceleration = sourceBasedBrakeAcceleration(vehicle);
        double idealStop = speed * speed / Math.max(0.01D, 2.0D * deceleration);
        double reactionMargin = speed * 2.0D + 0.35D;
        double required = ARRIVE_RADIUS + turnPenalty + idealStop + reactionMargin;
        return distance <= required;
    }

    private static double sourceBasedBrakeAcceleration(Entity vehicle) {
        if (isTrackedVehicle(vehicle)) {
            return Math.max(0.01D, readDouble(readMember(vehicle, "brakeAcceleration"), 0.025D));
        }
        return Math.max(0.01D, readDouble(readMember(vehicle, "brakeForce"), 0.025D));
    }

    private static boolean shouldHardBrakeNearFinal(Entity vehicle, Vec3 finalTarget, double distance, double speed, double captureDistance) {
        if (finalTarget == null || speed < 0.05D) return false;
        double effective = Math.max(ARRIVE_RADIUS + 1.0D, captureDistance + 0.50D);
        if (hasPassedFinalTarget(vehicle, finalTarget) && distance <= effective) return true;
        if (distance > ARRIVE_RADIUS + 1.25D) return false;
        Vec3 toTarget = finalTarget.subtract(vehicle.position()).multiply(1.0D, 0.0D, 1.0D);
        Vec3 velocity = vehicle.getDeltaMovement().multiply(1.0D, 0.0D, 1.0D);
        if (toTarget.lengthSqr() < 1.0E-6D || velocity.lengthSqr() < 1.0E-6D) return false;
        double closingPerTick = velocity.dot(toTarget.normalize());
        return closingPerTick > 0.02D && distance - closingPerTick * 2.0D <= ARRIVE_RADIUS + 0.05D;
    }

    private static double finalCaptureDistance(VehicleShape shape, double speed) {
        return Mth.clamp(2.0D + shape.radius() * 0.35D + Math.max(0.0D, speed - 0.05D) * 10.0D, 2.25D, 6.0D);
    }

    private static double finalCaptureHoldDistance(VehicleShape shape) {
        return Math.max(6.0D, shape.radius() * 2.0D + 1.0D);
    }

    private static boolean shouldCaptureFinalTarget(Entity vehicle, Vec3 finalTarget, double distance, double speed, VehicleShape shape) {
        if (distance <= ARRIVE_RADIUS) return true;
        double effective = finalCaptureDistance(shape, speed);
        CompoundTag data = vehicle.getPersistentData();
        if (distance <= effective) {
            long now = vehicle.level().getGameTime();
            if (!data.contains(EFFECTIVE_ARRIVE_SINCE)) {
                data.putLong(EFFECTIVE_ARRIVE_SINCE, now);
                return false;
            }
            return now - data.getLong(EFFECTIVE_ARRIVE_SINCE) >= EFFECTIVE_ARRIVAL_TIMEOUT_TICKS;
        }
        data.remove(EFFECTIVE_ARRIVE_SINCE);
        return false;
    }

    private static void captureFinalTarget(Entity vehicle, Vec3 finalTarget) {
        CompoundTag data = vehicle.getPersistentData();
        data.putDouble(CAPTURED_TARGET_X, finalTarget.x);
        data.putDouble(CAPTURED_TARGET_Z, finalTarget.z);
        data.remove(EFFECTIVE_ARRIVE_SINCE);
    }

    private static boolean hasCapturedFinalTarget(Entity vehicle, Vec3 finalTarget, VehicleShape shape) {
        CompoundTag data = vehicle.getPersistentData();
        if (!data.contains(CAPTURED_TARGET_X) || !data.contains(CAPTURED_TARGET_Z)) return false;
        double dx = finalTarget.x - data.getDouble(CAPTURED_TARGET_X);
        double dz = finalTarget.z - data.getDouble(CAPTURED_TARGET_Z);
        if (dx * dx + dz * dz > TARGET_SOFT_CHANGE_DISTANCE * TARGET_SOFT_CHANGE_DISTANCE) {
            data.remove(CAPTURED_TARGET_X);
            data.remove(CAPTURED_TARGET_Z);
            return false;
        }
        if (flatDistance(vehicle.position(), finalTarget) <= finalCaptureHoldDistance(shape)) return true;
        data.remove(CAPTURED_TARGET_X);
        data.remove(CAPTURED_TARGET_Z);
        return false;
    }

    private static boolean hasPassedFinalTarget(Entity vehicle, Vec3 finalTarget) {
        Vec3 toTarget = finalTarget.subtract(vehicle.position()).multiply(1.0D, 0.0D, 1.0D);
        Vec3 velocity = vehicle.getDeltaMovement().multiply(1.0D, 0.0D, 1.0D);
        if (toTarget.lengthSqr() < 1.0E-6D || velocity.lengthSqr() < 1.0E-6D) return false;
        return velocity.dot(toTarget.normalize()) < -0.02D;
    }

    private static boolean brakePulse(Entity vehicle, double distance, double speed) {
        if (distance < 2.4D || speed > 0.23D) return true;
        long tick = vehicle.level().getGameTime();
        return (tick % 5L) < 2L;
    }

    private enum DriveMode {
        FORWARD,
        REVERSE_SHORT,
        TURN_AROUND,
        THREE_POINT
    }

    private record DriveDecision(DriveMode mode, double forwardEta, double reverseEta, double turnAroundEta) {}

    private static DriveDecision decideDriveMode(Entity vehicle, float yawDiff, double absYaw, double distance, double speed, VehicleShape shape, boolean finalTarget) {
        double turnRadius = estimatedTurnRadius(vehicle, shape);
        boolean rearTarget = absYaw >= REVERSE_ANGLE;
        boolean tracked = isTrackedVehicle(vehicle);
        boolean shortReverseTarget = isShortReverseTarget(absYaw, distance, shape);
        boolean threePointCooldown = vehicle.level().getGameTime() < vehicle.getPersistentData().getLong(THREE_POINT_GIVE_UP_UNTIL);
        boolean cannotArc = shouldUseThreePointTurn(!tracked, shortReverseTarget, threePointCooldown,
                finalTarget, cannotArcToTarget(vehicle, absYaw, distance, shape));
        double turnPenalty = absYaw / 8.0D;
        double stopPenalty = Math.max(0.0D, speed) * 18.0D;
        double forwardEta = distance / 0.10D + turnPenalty + stopPenalty;
        if (cannotArc) forwardEta += 80.0D;
        double reverseEta = distance / 0.06D + Math.abs(Mth.wrapDegrees(yawDiff - 180.0F)) / 8.0D + stopPenalty * 1.15D;
        double turnAroundEta = stopPenalty + 50.0D + distance / 0.10D;
        // Do not infer a reverse manoeuvre from a waypoint's relative angle. A waypoint is
        // normally ahead on the drivable corridor even if it is not yet inside the current
        // turning circle. Native collision/stuck recovery is the only valid reason to back up.
        return new DriveDecision(DriveMode.FORWARD, forwardEta, reverseEta, turnAroundEta);
    }

    /** A route waypoint is a steering hint; reserve a K-turn for a real final target. */
    static boolean shouldUseThreePointTurn(boolean wheeled, boolean shortReverseTarget, boolean threePointCooldown,
                                           boolean finalTarget, boolean cannotArcToTarget) {
        return false;
    }

    /**
     * A tracked chassis can rotate without translating.  Preserve ordinary rolling steering
     * for small corrections and distant targets, but align it before entering a close corner.
     */
    static boolean shouldPivotTrackedVehicle(boolean tracked, double absYaw, double distance, double pivotRange,
                                             boolean intermediateWaypoint) {
        if (!tracked || absYaw < TURN_IN_PLACE_ANGLE) return false;
        if (distance <= pivotRange) return true;
        // A generated intermediate point is normally the mouth of a constrained route.  A
        // right-angle turn may safely be prepared a little earlier, while distant targets
        // still use normal forward steering.
        return intermediateWaypoint && absYaw >= TRACKED_PIVOT_CORNER_ANGLE && distance <= pivotRange * 1.5D;
    }

    static boolean shouldAttemptTrackedEscapePivot(boolean tracked, boolean horizontalCollision) {
        return tracked && horizontalCollision;
    }

    /**
     * A tree or other narrow obstacle can block the nose even when moving it aside only needs
     * a small change of heading.  Keep an escape heading briefly so the chassis cannot alternate
     * left/right every planner tick, then let ordinary forward steering resume.
     */
    private static float trackedEscapePivotYaw(Entity vehicle, Vec3 driveTarget, VehicleShape shape) {
        CompoundTag data = vehicle.getPersistentData();
        long now = vehicle.level().getGameTime();
        if (data.contains(TRACKED_ESCAPE_PIVOT_YAW) && now <= data.getLong(TRACKED_ESCAPE_PIVOT_UNTIL)) {
            float storedYaw = data.getFloat(TRACKED_ESCAPE_PIVOT_YAW);
            if (Math.abs(Mth.wrapDegrees(storedYaw - vehicle.getYRot())) > 4.0F) return storedYaw;
            clearTrackedEscapePivot(data);
        } else {
            clearTrackedEscapePivot(data);
        }
        if (!shouldAttemptTrackedEscapePivot(isTrackedVehicle(vehicle), vehicle.horizontalCollision)) return Float.NaN;

        float chosenYaw = chooseTrackedEscapeYaw(vehicle, driveTarget, shape);
        if (!Float.isFinite(chosenYaw)) return Float.NaN;
        data.putFloat(TRACKED_ESCAPE_PIVOT_YAW, chosenYaw);
        data.putLong(TRACKED_ESCAPE_PIVOT_UNTIL, now + TRACKED_ESCAPE_PIVOT_TICKS);
        return chosenYaw;
    }

    private static float chooseTrackedEscapeYaw(Entity vehicle, Vec3 driveTarget, VehicleShape shape) {
        float desiredYaw = yawTo(driveTarget.subtract(vehicle.position()).multiply(1.0D, 0.0D, 1.0D));
        float currentYaw = vehicle.getYRot();
        double probeDistance = Mth.clamp(shape.radius() + 2.0D, 3.0D, 8.0D);
        double[] offsets = {0.0D, 30.0D, -30.0D, 50.0D, -50.0D, 70.0D, -70.0D};
        for (double offset : offsets) {
            float candidateYaw = Mth.wrapDegrees(desiredYaw + (float) offset);
            if (Math.abs(Mth.wrapDegrees(candidateYaw - currentYaw)) < TRACKED_ESCAPE_MIN_ANGLE) continue;
            Vec3 forward = Vec3.directionFromRotation(0.0F, candidateYaw).multiply(1.0D, 0.0D, 1.0D).normalize();
            Vec3 probe = vehicle.position().add(forward.scale(probeDistance));
            if (canTravelDirect(vehicle, vehicle.position(), probe, shape, probeDistance)) return candidateYaw;
        }
        return Float.NaN;
    }

    private static void clearTrackedEscapePivot(CompoundTag data) {
        data.remove(TRACKED_ESCAPE_PIVOT_YAW);
        data.remove(TRACKED_ESCAPE_PIVOT_UNTIL);
    }

    private static double trackedPivotRange(VehicleShape shape) {
        return Mth.clamp(shape.radius() * 3.0D + 8.0D, 12.0D, 24.0D);
    }

    private static double estimatedTurnRadius(Entity vehicle, VehicleShape shape) {
        double maxTurn = Math.abs(readDouble(readMember(vehicle, "maxTurn"), 2.0D));
        double turnRadians = Math.toRadians(Math.max(1.0D, maxTurn));
        return Math.max(shape.radius() * 2.0D, shape.radius() / Math.max(0.05D, turnRadians));
    }

    private static boolean isShortReverseTarget(double absYaw, double distance, VehicleShape shape) {
        double reverseRange = Math.max(25.0D, shape.radius() * 4.0D);
        return distance <= reverseRange && absYaw >= SIDE_REVERSE_ANGLE;
    }

    private static boolean cannotArcToTarget(Entity vehicle, double absYaw, double distance, VehicleShape shape) {
        double turnRadius = estimatedTurnRadius(vehicle, shape);
        double cannotArcDistance = Math.min(35.0D, Math.max(12.0D, turnRadius * 2.0D));
        return absYaw >= 45.0D && distance <= cannotArcDistance;
    }

    private static boolean canExitThreePoint(double absYaw) {
        return absYaw <= 30.0D;
    }

    private static boolean changedThreePointTarget(CompoundTag data, Vec3 target) {
        if (target == null || !data.contains(THREE_POINT_TARGET_X) || !data.contains(THREE_POINT_TARGET_Z)) return false;
        double dx = target.x - data.getDouble(THREE_POINT_TARGET_X);
        double dz = target.z - data.getDouble(THREE_POINT_TARGET_Z);
        return dx * dx + dz * dz > TARGET_SOFT_CHANGE_DISTANCE * TARGET_SOFT_CHANGE_DISTANCE;
    }

    private static void clearThreePointState(Entity vehicle) {
        CompoundTag data = vehicle.getPersistentData();
        data.remove(THREE_POINT_STEP);
        data.remove(THREE_POINT_TICKS);
        data.remove(THREE_POINT_STEER);
        data.remove(THREE_POINT_LAST_X);
        data.remove(THREE_POINT_LAST_Z);
        data.remove(THREE_POINT_LAST_ABS_YAW);
        data.remove(THREE_POINT_STUCK_TICKS);
        data.remove(THREE_POINT_TARGET_X);
        data.remove(THREE_POINT_TARGET_Z);
    }

    private static short shortReverseKeys(float yawDiff, double absYaw) {
        boolean right = false;
        boolean left = false;
        if (absYaw < 170.0D) {
            // Backing up reverses steering response, so steer opposite the forward yaw error.
            right = yawDiff < 0.0F;
            left = yawDiff > 0.0F;
        }
        return packControl(false, true, right, left, false);
    }

    private static short threePointKeys(Entity vehicle, float yawDiff, double absYaw, double finalDistance, double speed, VehicleShape shape, Vec3 finalTarget) {
        CompoundTag data = vehicle.getPersistentData();
        boolean active = data.contains(THREE_POINT_STEP);
        if (active && changedThreePointTarget(data, finalTarget)) {
            clearThreePointState(vehicle);
            active = false;
        }
        if (isTrackedVehicle(vehicle)) return Short.MIN_VALUE;
        if (!active && speed > 0.16D) return packControl(false, false, false, false, true);
        if (!data.contains(THREE_POINT_STEP)) {
            data.putInt(THREE_POINT_STEP, 0);
            data.putInt(THREE_POINT_TICKS, 0);
            data.putInt(THREE_POINT_STEER, yawDiff >= 0.0F ? 1 : -1);
            data.putDouble(THREE_POINT_TARGET_X, finalTarget.x);
            data.putDouble(THREE_POINT_TARGET_Z, finalTarget.z);
        }
        int ticks = data.getInt(THREE_POINT_TICKS);
        if (canExitThreePoint(absYaw) || ticks >= MAX_THREE_POINT_TICKS) {
            clearThreePointState(vehicle);
            return Short.MIN_VALUE;
        }
        updateThreePointProgress(vehicle, data, absYaw);
        if (data.getInt(THREE_POINT_STUCK_TICKS) >= THREE_POINT_GIVE_UP_STUCK_TICKS) {
            data.putLong(THREE_POINT_GIVE_UP_UNTIL, vehicle.level().getGameTime() + THREE_POINT_GIVE_UP_COOLDOWN_TICKS);
            clearThreePointState(vehicle);
            return Short.MIN_VALUE;
        }
        int phase = data.getInt(THREE_POINT_STEP);
        ticks = data.getInt(THREE_POINT_TICKS);
        if (YwzjVehicleCompatConfig.multiStageKTurnEnabled() && phase == THREE_POINT_FORWARD_PHASE && data.getInt(THREE_POINT_STUCK_TICKS) == 0 && ticks > THREE_POINT_FORWARD_PHASE_TICKS) {
            setThreePointPhase(data, THREE_POINT_REVERSE_PHASE, vehicle, absYaw);
            phase = THREE_POINT_REVERSE_PHASE;
        }
        int steer = data.getInt(THREE_POINT_STEER);
        int appliedSteer = phase == THREE_POINT_FORWARD_PHASE ? steer : -steer;
        boolean right = appliedSteer > 0;
        boolean left = appliedSteer < 0;
        data.putInt(THREE_POINT_TICKS, ticks + 1);
        return packControl(phase == THREE_POINT_FORWARD_PHASE, phase == THREE_POINT_REVERSE_PHASE, right, left, false);
    }

    private static void updateThreePointProgress(Entity vehicle, CompoundTag data, double absYaw) {
        if (!data.contains(THREE_POINT_LAST_X)) {
            setThreePointPhase(data, data.getInt(THREE_POINT_STEP), vehicle, absYaw);
            return;
        }
        double dx = vehicle.getX() - data.getDouble(THREE_POINT_LAST_X);
        double dz = vehicle.getZ() - data.getDouble(THREE_POINT_LAST_Z);
        double movedSqr = dx * dx + dz * dz;
        double lastAbsYaw = data.getDouble(THREE_POINT_LAST_ABS_YAW);
        boolean yawImproved = absYaw < lastAbsYaw - 2.0D;
        boolean stalled = movedSqr < 0.0025D && !yawImproved;
        int stuck = stalled ? data.getInt(THREE_POINT_STUCK_TICKS) + 1 : 0;
        data.putInt(THREE_POINT_STUCK_TICKS, stuck);
        if (YwzjVehicleCompatConfig.multiStageKTurnEnabled() && stuck >= THREE_POINT_PHASE_STUCK_TICKS) {
            int next = data.getInt(THREE_POINT_STEP) == THREE_POINT_REVERSE_PHASE ? THREE_POINT_FORWARD_PHASE : THREE_POINT_REVERSE_PHASE;
            setThreePointPhase(data, next, vehicle, absYaw);
            return;
        }
        data.putDouble(THREE_POINT_LAST_X, vehicle.getX());
        data.putDouble(THREE_POINT_LAST_Z, vehicle.getZ());
        data.putDouble(THREE_POINT_LAST_ABS_YAW, absYaw);
    }

    private static void setThreePointPhase(CompoundTag data, int phase, Entity vehicle, double absYaw) {
        data.putInt(THREE_POINT_STEP, phase);
        data.putInt(THREE_POINT_TICKS, 0);
        data.putInt(THREE_POINT_STUCK_TICKS, 0);
        data.putDouble(THREE_POINT_LAST_X, vehicle.getX());
        data.putDouble(THREE_POINT_LAST_Z, vehicle.getZ());
        data.putDouble(THREE_POINT_LAST_ABS_YAW, absYaw);
    }

    private static short packControl(boolean forward, boolean backward, boolean right, boolean left, boolean brake) {
        short keys = 0;
        if (forward) keys |= 1;
        if (backward) keys |= 2;
        if (right) keys |= 4;
        if (left) keys |= 8;
        if (brake) keys |= 16;
        return keys;
    }

    private static void applyPackedControl(Entity vehicle, short keys) {
        applyControl(vehicle, (keys & 1) != 0, (keys & 2) != 0, (keys & 4) != 0, (keys & 8) != 0, (keys & 16) != 0);
    }

    private static void applyControl(Entity vehicle, boolean forward, boolean backward, boolean right, boolean left, boolean brake) {
        if (!(vehicle instanceof AbstractVehicle ywzjVehicle)) return;
        writeControl(ywzjVehicle, forward, backward, right, left, brake);
        registerActiveGroundControl(vehicle, forward, backward, right, left, brake);
        alignMainWeaponsForward(vehicle, forward);
    }

    private static void writeControl(AbstractVehicle vehicle, boolean forward, boolean backward, boolean right, boolean left, boolean brake) {
        ControlUnit control = vehicle.controlUnit;
        control.forward = forward;
        control.backward = backward;
        control.right = right;
        control.left = left;
        control.up = brake;
        control.down = false;
        control.leftYaw = false;
        control.rightYaw = false;
        control.functionalUp = false;
        control.functionalDown = false;
        control.functionalLeft = false;
        control.functionalRight = false;
        control.xRotKeep = false;
        control.yRotKeep = false;
    }

    private static void traceGroundControl(Entity vehicle, AbstractVehicle chassis, GroundControlState control) {
        long now = vehicle.level().getGameTime();
        Long previous = GROUND_CONTROL_TRACE_TICKS.get(vehicle.getUUID());
        if (previous != null && now - previous < 20L) return;
        GROUND_CONTROL_TRACE_TICKS.put(vehicle.getUUID(), now);
        pathDebug(vehicle, "GROUND_TICK_CONTROL", "tracked=%s forward=%s backward=%s right=%s left=%s brake=%s power=%.1f engine=%s",
                isTrackedVehicle(vehicle),
                control.forward(), control.backward(), control.right(), control.left(), control.brake(),
                chassis.getPower(), chassis.isEngineOn());
    }

    private static void alignMainWeaponsForward(Entity vehicle, boolean moving) {
        if (!moving) return;
        alignMainWeaponsToYaw(vehicle, vehicle.getYRot(), true);
    }

    private static float trackedWeaponAimYaw(Entity vehicle, TrackedPoseRoute route,
                                             TrackedPoseRouteBuild build, Vec3 fallbackTarget) {
        if (route != null && route.index >= 0 && route.index < route.steps.size()) {
            return route.steps.get(route.index).yaw;
        }
        Vec3 target = build != null ? build.commandTarget : fallbackTarget;
        if (target != null) {
            Vec3 horizontal = target.subtract(vehicle.position()).multiply(1.0D, 0.0D, 1.0D);
            if (horizontal.lengthSqr() > 1.0E-6D) return yawTo(horizontal);
        }
        return vehicle.getYRot();
    }

    private static void alignMainWeaponsToYaw(Entity vehicle, float yaw, boolean active) {
        if (!active) return;
        Vec3 direction = Vec3.directionFromRotation(0.0F, yaw).multiply(1.0D, 0.0D, 1.0D).normalize();
        Vec3 front = vehicle.position().add(direction.scale(24.0D));
        List<?> partUnits = asList(invokeNoArg(vehicle, "getPartUnits"));
        if (partUnits == null) return;
        for (Object partUnit : partUnits) alignWeaponTreeForward(vehicle, partUnit, front);
    }

    private static void alignWeaponTreeForward(Entity vehicle, Object partUnit, Vec3 front) {
        if (partUnit == null) return;
        if (isInstance(partUnit, WEAPON_UNIT_CLASS_NAME)) {
            invoke(partUnit, "setLockedEntity", new Class<?>[]{Entity.class}, new Object[]{null});
            invoke(partUnit, "aim", new Class<?>[]{Vec3.class}, front);
        }
        List<?> subs = asList(invokeNoArg(partUnit, "getSubWeaponUnits"));
        if (subs != null) for (Object sub : subs) alignWeaponTreeForward(vehicle, sub, front);
    }

    private static void stopVehicle(Entity vehicle) {
        unregisterActiveGroundControl(vehicle);
        clearTrackedEscapePivot(vehicle.getPersistentData());
        if (vehicle instanceof AbstractVehicle ywzjVehicle) ywzjVehicle.controlUnit.reset();
    }

    private record GroundControlState(boolean forward, boolean backward, boolean right, boolean left, boolean brake) {}

    private static void pathDebug(Entity vehicle, String phase, String format, Object... args) {
        if (verbosePathPhase(phase) && !YwzjVehicleCompatConfig.pathTraceVerbose()) return;
        try {
            String message = args == null || args.length == 0 ? format : String.format(Locale.ROOT, format, args);
            LOGGER.info("{}", String.format(Locale.ROOT, "[DS-YWZJ-PATH] phase=%s vehicle=%s pos=%s yaw=%.1f speed=%.3f %s",
                    phase,
                    vehicle == null ? "null" : vehicle.getStringUUID(),
                    vehicle == null ? "null" : fmt(vehicle.position()),
                    vehicle == null ? 0.0F : vehicle.getYRot(),
                    vehicle == null ? 0.0D : horizontalSpeed(vehicle),
                    message));
        } catch (RuntimeException ignored) {
        }
    }

    private static boolean verbosePathPhase(String phase) {
        if (phase == null) return false;
        return phase.equals("POSE_EXPAND")
                || phase.startsWith("POSE_CANDIDATE_")
                || phase.startsWith("POSE_NODE_")
                || phase.startsWith("POSE_OCCUPY_")
                || phase.startsWith("POSE_RESOLVE_")
                || phase.startsWith("POSE_SWEEP_")
                || phase.equals("POSE_ROUTE_STEP")
                || phase.equals("POSE_SHORTCUT_KEEP")
                || phase.equals("POSE_CONTROL_TICK")
                || phase.equals("POSE_MOTION_WATCH")
                || phase.startsWith("POSE_DEPENETRATE_CANDIDATE_")
                || phase.equals("GROUND_TICK_CONTROL");
    }

    private static String fmt(Vec3 vec) {
        if (vec == null) return "null";
        return String.format(Locale.ROOT, "(%.1f,%.1f,%.1f)", vec.x, vec.y, vec.z);
    }

    private static LivingEntity driver(Entity vehicle) {
        return vehicle instanceof AbstractVehicle ywzjVehicle ? ywzjVehicle.getDriver() : null;
    }

    private static Entity seatPassenger(Entity vehicle, int seatIndex) {
        if (!(vehicle.level() instanceof ServerLevel level)) return null;
        List<?> seats = asList(readMember(vehicle, "seats"));
        if (seats == null) return null;
        for (Object seat : seats) {
            if (readInt(readMember(seat, "seatIndex"), -1) != seatIndex) continue;
            int id = readInt(readMember(seat, "passengerId"), -1);
            return id >= 0 ? level.getEntity(id) : null;
        }
        return null;
    }

    private static String seatType(int index, Object partUnit) {
        if (index == 0) return "驾驶";
        if (isInstance(partUnit, WEAPON_UNIT_CLASS_NAME)) return "炮位";
        Object id = readMember(partUnit, "id");
        String raw = id instanceof String s ? s : "";
        if (raw.toLowerCase(Locale.ROOT).contains("weapon") || raw.toLowerCase(Locale.ROOT).contains("gun")) return "炮位";
        return "座位 " + (index + 1);
    }

    private static boolean isAirVehicle(Entity vehicle) {
        return vehicle instanceof FixedWingVehicle || vehicle instanceof RotaryWingVehicle;
    }

    private static boolean isRotaryWingVehicle(Entity vehicle) {
        return vehicle instanceof RotaryWingVehicle;
    }

    private static boolean isHelicopterFlying(Entity vehicle) {
        return vehicle != null && (!vehicle.onGround() || readFloat(invokeNoArg(vehicle, "getPower"), 0.0F) > 1.0F);
    }

    private static double helicopterBottomY(Entity vehicle) {
        AABB mainBody = mainObbAabb(vehicle);
        return mainBody == null ? vehicle.getBoundingBox().minY : mainBody.minY;
    }

    /**
     * Vehicle position is not its landing gear height. Z-10's origin is roughly
     * five blocks above the ground while resting, so targeting terrainY directly
     * makes a landed helicopter keep demanding descent forever.
     */
    private static double helicopterLandingOriginY(Entity vehicle, double terrainY) {
        double originToBottom = Math.max(0.0D, vehicle.getY() - helicopterBottomY(vehicle));
        return terrainY + originToBottom + 0.08D;
    }

    private static double groundProjectionY(Entity vehicle, double x, double z) {
        if (vehicle == null || vehicle.level() == null) return 0.0D;
        Level level = vehicle.level();
        int blockX = Mth.floor(x), blockZ = Mth.floor(z);
        // Client heightmaps can temporarily report min build height and cannot
        // distinguish a roof/sky island above the aircraft from ground below it.
        // Project from the aircraft downward and skip whole empty sections, so the
        // always-visible command ring remains correct in every 3D environment.
        BlockPos.MutableBlockPos cursor = BlockPos.containing(x, vehicle.getY(), z).mutable();
        LevelChunk chunk = level.getChunk(blockX >> 4, blockZ >> 4);
        LevelChunkSection[] sections = chunk.getSections();
        int y = Math.min(cursor.getY(), level.getMaxBuildHeight() - 1);
        while (y >= level.getMinBuildHeight()) {
            int sectionIndex = chunk.getSectionIndex(y);
            if (sectionIndex >= 0 && sectionIndex < sections.length && sections[sectionIndex].hasOnlyAir()) {
                y = SectionPos.sectionToBlockCoord(SectionPos.blockToSectionCoord(y)) - 1;
                continue;
            }
            cursor.setY(y);
            BlockState state = level.getBlockState(cursor);
            VoxelShape shape = state.getCollisionShape(level, cursor);
            if (!state.is(Blocks.BARRIER) && !shape.isEmpty()) {
                return y + shape.max(net.minecraft.core.Direction.Axis.Y);
            }
            y--;
        }
        return level.getMinBuildHeight();
    }

    /**
     * Keep a large projected rotor ring above uneven terrain. A center-only sample
     * can land in a shallow hole and bury the whole marker even when the remaining
     * footprint is on the surrounding floor.
     */
    private static double projectedFootprintGroundY(Entity vehicle, AABB footprint) {
        double centerX = footprint.getCenter().x;
        double centerZ = footprint.getCenter().z;
        double insetX = Math.min(2.0D, footprint.getXsize() * 0.2D);
        double insetZ = Math.min(2.0D, footprint.getZsize() * 0.2D);
        double minX = Math.min(centerX, footprint.minX + insetX);
        double maxX = Math.max(centerX, footprint.maxX - insetX);
        double minZ = Math.min(centerZ, footprint.minZ + insetZ);
        double maxZ = Math.max(centerZ, footprint.maxZ - insetZ);
        double highest = groundProjectionY(vehicle, centerX, centerZ);
        highest = Math.max(highest, groundProjectionY(vehicle, minX, centerZ));
        highest = Math.max(highest, groundProjectionY(vehicle, maxX, centerZ));
        highest = Math.max(highest, groundProjectionY(vehicle, centerX, minZ));
        highest = Math.max(highest, groundProjectionY(vehicle, centerX, maxZ));
        return highest;
    }

    private static boolean hasAnyPassenger(Entity vehicle) {
        if (vehicle == null) return false;
        if (!vehicle.getPassengers().isEmpty()) return true;
        if (vehicle instanceof AbstractVehicle ywzjVehicle && ywzjVehicle.seats != null) {
            for (AbstractVehicle.Seat seat : ywzjVehicle.seats) {
                if (seat != null && seat.passengerId != null && seat.passengerId >= 0) return true;
            }
        }
        return false;
    }

    private static double flightClearanceGroundY(Entity vehicle, Vec3 target) {
        if (!(vehicle.level() instanceof ServerLevel level)) return target.y;
        CompoundTag data = vehicle.getPersistentData();
        long now = level.getGameTime();
        double dx = target.x - data.getDouble(HELI_CLEARANCE_SCAN_TARGET_X);
        double dz = target.z - data.getDouble(HELI_CLEARANCE_SCAN_TARGET_Z);
        if (data.contains(HELI_CLEARANCE_SCAN_GROUND_Y)
                && now - data.getLong(HELI_CLEARANCE_SCAN_TICK) < HELI_CLEARANCE_SCAN_INTERVAL
                && dx * dx + dz * dz <= 1.0D) return data.getDouble(HELI_CLEARANCE_SCAN_GROUND_Y);
        Vec3 horizontal = target.subtract(vehicle.position()).multiply(1.0D, 0.0D, 1.0D);
        double distance = Math.min(HELI_CLEARANCE_SCAN_RANGE, horizontal.length());
        int samples = Math.max(1, Mth.ceil(distance / HELI_CLEARANCE_SCAN_STEP));
        double highest = groundProjectionY(vehicle, vehicle.getX(), vehicle.getZ());
        for (int index = 1; index <= samples; index++) {
            double t = (double) index / samples;
            double x = vehicle.getX() + horizontal.x * t;
            double z = vehicle.getZ() + horizontal.z * t;
            highest = Math.max(highest, level.getHeight(Heightmap.Types.MOTION_BLOCKING_NO_LEAVES, Mth.floor(x), Mth.floor(z)));
        }
        data.putLong(HELI_CLEARANCE_SCAN_TICK, now);
        data.putDouble(HELI_CLEARANCE_SCAN_TARGET_X, target.x);
        data.putDouble(HELI_CLEARANCE_SCAN_TARGET_Z, target.z);
        data.putDouble(HELI_CLEARANCE_SCAN_GROUND_Y, highest);
        return highest;
    }

    private static double boundedAbsoluteHelicopterAltitude(Entity vehicle, double requestedY) {
        if (!(vehicle.level() instanceof ServerLevel level)) return Mth.clamp(requestedY, -64.0D, 500.0D);
        double floorY = groundProjectionY(vehicle, vehicle.getX(), vehicle.getZ());
        double minimum = floorY + 3.0D;
        int upperLimit = Math.min(500, level.getMaxBuildHeight() - 4);
        double maximum = upperLimit;
        HitResult ceiling = level.clip(new ClipContext(new Vec3(vehicle.getX(), minimum, vehicle.getZ()),
                new Vec3(vehicle.getX(), upperLimit, vehicle.getZ()), ClipContext.Block.COLLIDER, ClipContext.Fluid.NONE, vehicle));
        if (ceiling.getType() != HitResult.Type.MISS) maximum = Math.floor(ceiling.getLocation().y) - 3.0D;
        // A gap too small for the aircraft has no safe locked altitude; stay at the only non-ceiling boundary.
        if (maximum < minimum) return maximum;
        return Mth.clamp(requestedY, minimum, maximum);
    }

    /** Fixed altitude means buildings are horizontal obstacles, never a reason to climb. */
    private static Vec3 helicopterObstacleAvoidanceTarget(Entity vehicle, Vec3 finalTarget) {
        if (!(vehicle.level() instanceof ServerLevel level)) return finalTarget;
        CompoundTag data = vehicle.getPersistentData();
        double safetyRadius = helicopterSafetyRadius(vehicle);
        double safetyHalfHeight = helicopterSafetyHalfHeight(vehicle);
        if (helicopterCorridorClear(vehicle, vehicle.position(), finalTarget, safetyRadius, safetyHalfHeight)) {
            data.remove(HELI_AVOID_EXPIRES);
            return finalTarget;
        }
        long now = level.getGameTime();
        double targetDx = finalTarget.x - data.getDouble(HELI_AVOID_TARGET_X);
        double targetDz = finalTarget.z - data.getDouble(HELI_AVOID_TARGET_Z);
        if (data.contains(HELI_AVOID_EXPIRES) && now <= data.getLong(HELI_AVOID_EXPIRES)
                && targetDx * targetDx + targetDz * targetDz <= 1.0D) {
            Vec3 waypoint = new Vec3(data.getDouble(HELI_AVOID_WAYPOINT_X), finalTarget.y, data.getDouble(HELI_AVOID_WAYPOINT_Z));
            if (vehicle.position().distanceToSqr(waypoint) > 4.0D && helicopterCorridorClear(vehicle, vehicle.position(), waypoint, safetyRadius, safetyHalfHeight)) return waypoint;
        }
        Vec3 flat = finalTarget.subtract(vehicle.position()).multiply(1.0D, 0.0D, 1.0D);
        if (flat.lengthSqr() < 1.0E-4D) return finalTarget;
        Vec3 forward = flat.normalize();
        Vec3 right = new Vec3(-forward.z, 0.0D, forward.x);
        double offset = safetyRadius + 3.0D;
        Vec3 best = null;
        double bestScore = Double.POSITIVE_INFINITY;
        for (int side : new int[]{-1, 1}) {
            for (double distance : new double[]{offset, offset * 1.75D, offset * 2.5D}) {
                Vec3 candidate = vehicle.position().add(forward.scale(Math.max(4.0D, offset * 0.5D))).add(right.scale(side * distance));
                if (!helicopterCorridorClear(vehicle, vehicle.position(), candidate, safetyRadius, safetyHalfHeight)) continue;
                double score = candidate.distanceToSqr(finalTarget);
                if (score < bestScore) { best = candidate; bestScore = score; }
            }
        }
        if (best == null) return finalTarget;
        data.putDouble(HELI_AVOID_TARGET_X, finalTarget.x);
        data.putDouble(HELI_AVOID_TARGET_Z, finalTarget.z);
        data.putDouble(HELI_AVOID_WAYPOINT_X, best.x);
        data.putDouble(HELI_AVOID_WAYPOINT_Z, best.z);
        data.putLong(HELI_AVOID_EXPIRES, now + HELI_AVOID_TTL);
        return best;
    }

    private static boolean helicopterCorridorClear(Entity vehicle, Vec3 from, Vec3 to, double safetyRadius, double safetyHalfHeight) {
        if (!(vehicle.level() instanceof ServerLevel level)) return true;
        Vec3 flat = to.subtract(from).multiply(1.0D, 0.0D, 1.0D);
        if (flat.lengthSqr() < 1.0E-5D) return true;
        Vec3 right = new Vec3(-flat.z, 0.0D, flat.x).normalize();
        for (double lateral : HELI_OBSTACLE_LATERAL_SAMPLES) for (double vertical : HELI_OBSTACLE_VERTICAL_SAMPLES) {
            Vec3 offset = right.scale(lateral * safetyRadius).add(0.0D, vertical * safetyHalfHeight, 0.0D);
            HitResult hit = level.clip(new ClipContext(from.add(offset), to.add(offset), ClipContext.Block.COLLIDER, ClipContext.Fluid.NONE, vehicle));
            if (hit.getType() != HitResult.Type.MISS) return false;
        }
        return true;
    }

    private static double helicopterSafetyRadius(Entity vehicle) {
        AABB obb = allObbAabb(vehicle);
        double body = Math.max(vehicle.getBbWidth(), vehicle.getBbHeight());
        if (obb != null) body = Math.max(body, Math.max(obb.getXsize(), obb.getZsize()));
        return Math.max(8.0D, body + 6.0D);
    }

    private static double helicopterSafetyHalfHeight(Entity vehicle) {
        AABB obb = allObbAabb(vehicle);
        double halfHeight = vehicle.getBbHeight() * 0.5D;
        if (obb != null) halfHeight = Math.max(halfHeight, obb.getYsize() * 0.5D);
        return Math.max(3.0D, halfHeight + 2.5D);
    }

    private static boolean isTrackedVehicle(Entity vehicle) {
        return classNameContains(vehicle, ".TrackedVehicle");
    }

    private static boolean classNameContains(Object object, String namePart) {
        return object != null && CLASS_CONTAINS_CACHE.computeIfAbsent(new ClassNameKey(object.getClass(), namePart), key -> classNameContainsUncached(key.type(), key.name()));
    }

    private static boolean classNameContainsUncached(Class<?> objectClass, String namePart) {
        for (Class<?> type = objectClass; type != null; type = type.getSuperclass()) {
            if (type.getName().contains(namePart)) return true;
        }
        return false;
    }

    private static boolean isInstance(Object object, String className) {
        return object != null && CLASS_MATCH_CACHE.computeIfAbsent(new ClassNameKey(object.getClass(), className), key -> isInstanceUncached(key.type(), key.name()));
    }

    private static boolean isInstanceUncached(Class<?> objectClass, String className) {
        for (Class<?> type = objectClass; type != null; type = type.getSuperclass()) {
            if (type.getName().equals(className)) return true;
        }
        for (Class<?> type : objectClass.getInterfaces()) {
            if (type.getName().equals(className)) return true;
        }
        return false;
    }

    private static float yawTo(Vec3 flat) {
        return (float) (Mth.atan2(flat.z, flat.x) * Mth.RAD_TO_DEG) - 90.0F;
    }

    private static double horizontalSpeed(Entity vehicle) {
        Vec3 velocity = vehicle.getDeltaMovement();
        return Math.sqrt(velocity.x * velocity.x + velocity.z * velocity.z);
    }

    private static AABB allObbAabb(Entity vehicle) {
        Object rawObbs = invokeNoArg(vehicle, "getOBBs");
        List<?> obbs = asList(rawObbs);
        AABB result = null;
        if (obbs != null) for (Object obb : obbs) result = union(result, obbAabb(obb));
        Object cube = invokeNoArg(vehicle, "getMainCubeOBB");
        Object obb = cube == null ? null : invokeNoArg(cube, "obb");
        result = union(result, obbAabb(obb));
        return result;
    }

    private static AABB obbAabb(Object obb) {
        Object vertices = obb == null ? null : invokeNoArg(obb, "getVertices");
        if (!(vertices instanceof Object[] array) || array.length == 0) return null;
        double minX = Double.POSITIVE_INFINITY, minY = Double.POSITIVE_INFINITY, minZ = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY, maxZ = Double.NEGATIVE_INFINITY;
        for (Object vertex : array) {
            double x = readDouble(readMember(vertex, "x"), Double.NaN);
            double y = readDouble(readMember(vertex, "y"), Double.NaN);
            double z = readDouble(readMember(vertex, "z"), Double.NaN);
            if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(z)) continue;
            minX = Math.min(minX, x); minY = Math.min(minY, y); minZ = Math.min(minZ, z);
            maxX = Math.max(maxX, x); maxY = Math.max(maxY, y); maxZ = Math.max(maxZ, z);
        }
        return minX <= maxX ? new AABB(minX, minY, minZ, maxX, maxY, maxZ) : null;
    }

    private static AABB union(AABB a, AABB b) {
        if (a == null) return b;
        if (b == null) return a;
        return new AABB(Math.min(a.minX, b.minX), Math.min(a.minY, b.minY), Math.min(a.minZ, b.minZ),
                Math.max(a.maxX, b.maxX), Math.max(a.maxY, b.maxY), Math.max(a.maxZ, b.maxZ));
    }

    private static AABB mainObbAabb(Entity vehicle) {
        Object cube = invokeNoArg(vehicle, "getMainCubeOBB");
        Object obb = cube == null ? null : invokeNoArg(cube, "obb");
        return obbAabb(obb);
    }

    private static void traceRouteShape(Entity vehicle, VehicleShape shape) {
        AABB entityBox = vehicle.getBoundingBox();
        pathDebug(vehicle, "SHAPE", "wheeled=%s entity=%s physicsObb=%s allObb=%s routeBox=%s radius=%.2f exactNativeBox=%s",
                !isTrackedVehicle(vehicle), boxSize(entityBox), boxSize(mainObbAabb(vehicle)), boxSize(allObbAabb(vehicle)),
                boxSize(shape.aabbAt(vehicle.position())), shape.radius(), shape.exactNativeBox());
    }

    private static String boxSize(AABB box) {
        return box == null ? "none" : String.format(Locale.ROOT, "%.2fx%.2fx%.2f", box.getXsize(), box.getYsize(), box.getZsize());
    }

    private static String boxBounds(AABB box) {
        return box == null ? "none" : String.format(Locale.ROOT, "[%.2f,%.2f,%.2f -> %.2f,%.2f,%.2f]",
                box.minX, box.minY, box.minZ, box.maxX, box.maxY, box.maxZ);
    }

    private static AABB allComponentObbAabb(Entity vehicle) {
        AABB result = null;
        if (vehicle instanceof AbstractVehicle ywzjVehicle && ywzjVehicle.getVehicleCubeOBBs() != null) {
            for (Object cube : ywzjVehicle.getVehicleCubeOBBs()) {
                Object obb = cube == null ? null : invokeNoArg(cube, "obb");
                result = union(result, obbAabb(obb));
            }
        }
        return result == null ? mainObbAabb(vehicle) : result;
    }

    private static void traceHelicopterSelection(Entity vehicle, AABB projected, double groundY) {
        if (!YwzjVehicleCompatConfig.flightControlTraceEnabled() || vehicle == null || vehicle.level() == null || !vehicle.level().isClientSide) return;
        long now = vehicle.level().getGameTime();
        Long previous = HELICOPTER_SELECTION_TRACE_TICKS.get(vehicle.getUUID());
        if (previous != null && previous + 20L > now) return;
        HELICOPTER_SELECTION_TRACE_TICKS.put(vehicle.getUUID(), now);
        int components = vehicle instanceof AbstractVehicle ywzjVehicle && ywzjVehicle.getVehicleCubeOBBs() != null
                ? ywzjVehicle.getVehicleCubeOBBs().size() : -1;
        LOGGER.info("[DS-YWZJ-HELI-SELECT] tick={} vehicle={} class={} name={} vehicleId={} passengers={} seats={} components={} groundY={} projected={}",
                now, vehicle.getId(), vehicle.getClass().getName(), vehicle.getDisplayName().getString(),
                vehicle instanceof AbstractVehicle ywzjVehicle ? ywzjVehicle.getVehicleId() : "unknown",
                vehicle.getPassengers().size(),
                vehicle instanceof AbstractVehicle ywzjVehicle && ywzjVehicle.seats != null ? ywzjVehicle.seats.size() : -1,
                components, groundY, projected);
    }

    private static List<Vec3> mainObbTopCorners(Entity vehicle, double yOffset) {
        Object cube = invokeNoArg(vehicle, "getMainCubeOBB");
        Object obb = cube == null ? null : invokeNoArg(cube, "obb");
        Object vertices = obb == null ? null : invokeNoArg(obb, "getVertices");
        if (!(vertices instanceof Object[] array) || array.length < 4) return List.of();
        List<Vec3> points = new ArrayList<>();
        for (Object vertex : array) {
            double x = readDouble(readMember(vertex, "x"), Double.NaN);
            double y = readDouble(readMember(vertex, "y"), Double.NaN);
            double z = readDouble(readMember(vertex, "z"), Double.NaN);
            if (Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(z)) points.add(new Vec3(x, y, z));
        }
        if (points.size() < 4) return List.of();
        points.sort(Comparator.comparingDouble((Vec3 p) -> p.y).reversed());
        List<Vec3> top = new ArrayList<>(points.subList(0, 4));
        double cx = top.stream().mapToDouble(p -> p.x).average().orElse(vehicle.getX());
        double cz = top.stream().mapToDouble(p -> p.z).average().orElse(vehicle.getZ());
        top.sort(Comparator.comparingDouble(p -> Math.atan2(p.z - cz, p.x - cx)));
        return top.stream().map(p -> p.add(0.0D, yOffset, 0.0D)).toList();
    }

    /** Builds one rotated footprint around every component OBB instead of only the main chassis cube. */
    private static List<Vec3> completeObbTopCorners(Entity vehicle, double yOffset) {
        List<Vec3> main = mainObbTopCorners(vehicle, 0.0D);
        if (main.size() < 4) return List.of();
        Vec3 axisX = main.get(1).subtract(main.get(0)).multiply(1.0D, 0.0D, 1.0D);
        Vec3 second = main.get(3).subtract(main.get(0)).multiply(1.0D, 0.0D, 1.0D);
        if (axisX.lengthSqr() < 1.0E-8D || second.lengthSqr() < 1.0E-8D) return List.of();
        axisX = axisX.normalize();
        Vec3 axisZ = second.subtract(axisX.scale(second.dot(axisX)));
        if (axisZ.lengthSqr() < 1.0E-8D) axisZ = new Vec3(-axisX.z, 0.0D, axisX.x);
        else axisZ = axisZ.normalize();

        List<Vec3> points = new ArrayList<>();
        if (vehicle instanceof AbstractVehicle ywzjVehicle && ywzjVehicle.getVehicleCubeOBBs() != null) {
            for (Object cube : ywzjVehicle.getVehicleCubeOBBs()) appendObbVertices(points, cube == null ? null : invokeNoArg(cube, "obb"));
        }
        Object mainCube = invokeNoArg(vehicle, "getMainCubeOBB");
        appendObbVertices(points, mainCube == null ? null : invokeNoArg(mainCube, "obb"));
        AABB entityBounds = vehicle.getBoundingBox();
        points.addAll(List.of(
                new Vec3(entityBounds.minX, entityBounds.minY, entityBounds.minZ), new Vec3(entityBounds.maxX, entityBounds.minY, entityBounds.minZ),
                new Vec3(entityBounds.maxX, entityBounds.minY, entityBounds.maxZ), new Vec3(entityBounds.minX, entityBounds.minY, entityBounds.maxZ),
                new Vec3(entityBounds.minX, entityBounds.maxY, entityBounds.minZ), new Vec3(entityBounds.maxX, entityBounds.maxY, entityBounds.minZ),
                new Vec3(entityBounds.maxX, entityBounds.maxY, entityBounds.maxZ), new Vec3(entityBounds.minX, entityBounds.maxY, entityBounds.maxZ)));
        if (points.isEmpty()) return List.of();

        double minX = Double.POSITIVE_INFINITY, minZ = Double.POSITIVE_INFINITY, maxX = Double.NEGATIVE_INFINITY, maxZ = Double.NEGATIVE_INFINITY;
        double topY = Double.NEGATIVE_INFINITY;
        for (Vec3 point : points) {
            double x = point.dot(axisX), z = point.dot(axisZ);
            minX = Math.min(minX, x); maxX = Math.max(maxX, x);
            minZ = Math.min(minZ, z); maxZ = Math.max(maxZ, z);
            topY = Math.max(topY, point.y);
        }
        if (!Double.isFinite(minX) || !Double.isFinite(topY)) return List.of();
        double y = topY + yOffset;
        Vec3 minMin = axisX.scale(minX).add(axisZ.scale(minZ));
        Vec3 maxMin = axisX.scale(maxX).add(axisZ.scale(minZ));
        Vec3 maxMax = axisX.scale(maxX).add(axisZ.scale(maxZ));
        Vec3 minMax = axisX.scale(minX).add(axisZ.scale(maxZ));
        return List.of(
                new Vec3(minMin.x, y, minMin.z), new Vec3(maxMin.x, y, maxMin.z),
                new Vec3(maxMax.x, y, maxMax.z), new Vec3(minMax.x, y, minMax.z));
    }

    private static void appendObbVertices(List<Vec3> output, Object obb) {
        Object vertices = obb == null ? null : invokeNoArg(obb, "getVertices");
        if (!(vertices instanceof Object[] array)) return;
        for (Object vertex : array) {
            double x = readDouble(readMember(vertex, "x"), Double.NaN);
            double y = readDouble(readMember(vertex, "y"), Double.NaN);
            double z = readDouble(readMember(vertex, "z"), Double.NaN);
            if (Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(z)) output.add(new Vec3(x, y, z));
        }
    }

    private record SafeCandidate(Vec3 position, double score) {}
    private record FleetMove(int index, Entity vehicle, Vec3 target) {
        double distanceSqr() { return vehicle.distanceToSqr(target); }
    }
    private record TrackPoint(Vec3 position, long tick) {}
    private record TrackRoadKey(int x, int z, int dx, int dz) {}
    private record TrackRoadSegment(List<TrackPoint> points, long expiresAt) {}
    private record TrackRoadAdvance(Vec3 position, int index) {}
    private record TrackRoadMatch(List<TrackPoint> points, int closestIndex, TrackRoadKey key) {}
    private record TrackSafetyKey(UUID vehicle, int x, int z, int radius) {}
    private record CachedTrackSafety(boolean safe, long expiresAt) {}

    private static final class AsyncRouteBuild {
        final Vec3 start, safe;
        final VehicleShape shape;
        final Set<UUID> ignored;
        final long generation;
        final int goalX, goalZ;
        final List<DominionAsyncGridPlanner.Point> points = new ArrayList<>();
        final Map<Long, DominionAsyncGridPlanner.Cell> cells = new HashMap<>();
        int cursor;
        CompletableFuture<DominionAsyncGridPlanner.Result> future;

        AsyncRouteBuild(Vec3 start, Vec3 safe, VehicleShape shape, Set<UUID> ignored, long generation, int radius, int goalX, int goalZ) {
            this.start = start; this.safe = safe; this.shape = shape; this.ignored = ignored; this.generation = generation; this.goalX = goalX; this.goalZ = goalZ;
            points.add(new DominionAsyncGridPlanner.Point(0, 0));
            points.add(new DominionAsyncGridPlanner.Point(goalX, goalZ));
            for (int x = -radius; x <= radius; x++) for (int z = -radius; z <= radius; z++) {
                if ((x == 0 && z == 0) || (x == goalX && z == goalZ)) continue;
                points.add(new DominionAsyncGridPlanner.Point(x, z));
            }
        }
    }

    private record TrackedPoseKey(int x, int z, int heading) {}
    private record TrackedPoseCacheKey(int x2, int y2, int z2, int yaw5) {}

    private record TrackedPose(Vec3 position, float yaw, boolean reverse) {}

    private static final class TrackedPoseNode implements Comparable<TrackedPoseNode> {
        final TrackedPoseKey key;
        final Vec3 position;
        final TrackedPoseNode parent;
        final double cost;
        final double priority;
        final boolean reverse;

        TrackedPoseNode(TrackedPoseKey key, Vec3 position, TrackedPoseNode parent,
                        double cost, double priority, boolean reverse) {
            this.key = key;
            this.position = position;
            this.parent = parent;
            this.cost = cost;
            this.priority = priority;
            this.reverse = reverse;
        }

        @Override
        public int compareTo(TrackedPoseNode other) {
            return Double.compare(priority, other.priority);
        }
    }

    private static final class TrackedPoseRouteBuild {
        final Vec3 anchor;
        final Vec3 target;
        Vec3 commandTarget;
        final TrackedHull hull;
        final Set<UUID> ignored;
        final long generation;
        final float startYaw;
        final double range;
        final Map<TrackedPoseKey, Double> costs = new HashMap<>();
        final Map<TrackedPoseCacheKey, Boolean> poseCache = new HashMap<>();
        final PriorityQueue<TrackedPoseNode> open = new PriorityQueue<>();
        final long startedTick;
        long lastAdvanceTick = Long.MIN_VALUE;
        long lastProgressTick;
        long cpuNanos;
        long poseTests;
        long obbFallbacks;
        long rejectedPoses;
        long sweeps;
        long cacheHits;
        long convexRejects;
        long terrainContacts;
        TrackedPoseNode bestNode;
        int expanded;

        TrackedPoseRouteBuild(Vec3 start, float startYaw, Vec3 target, TrackedHull hull, Set<UUID> ignored,
                              long generation, double range, long startedTick) {
            this.anchor = start;
            this.target = target;
            this.commandTarget = target;
            this.hull = hull;
            this.ignored = ignored;
            this.generation = generation;
            this.startYaw = startYaw;
            this.range = range;
            this.startedTick = startedTick;
            this.lastProgressTick = startedTick - 20L;
            int heading = trackedPoseHeading(startYaw);
            TrackedPoseKey startKey = new TrackedPoseKey(0, 0, heading);
            TrackedPoseNode startNode = new TrackedPoseNode(startKey, start, null, 0.0D,
                    flatDistance(start, target), false);
            bestNode = startNode;
            costs.put(startKey, 0.0D);
            open.add(startNode);
        }

        boolean matches(Vec3 otherTarget) {
            return otherTarget != null
                    && flatDistanceSqr(target, otherTarget)
                    <= TRACKED_POSE_TARGET_REUSE_RADIUS * TRACKED_POSE_TARGET_REUSE_RADIUS;
        }

        boolean matchesFinalTarget(CompoundTag data) {
            if (data.getLong(PATH_GENERATION) != generation || !data.contains(FINAL_TARGET_X) || !data.contains(FINAL_TARGET_Z)) return false;
            double x = data.getDouble(FINAL_TARGET_X), z = data.getDouble(FINAL_TARGET_Z);
            return (x - target.x) * (x - target.x) + (z - target.z) * (z - target.z) <= 4.0D;
        }

        boolean inRange(Vec3 position) {
            return flatDistance(anchor, position) <= range;
        }
    }

    private static final class TrackedPoseRoute {
        final Vec3 target;
        final Vec3 safe;
        final long generation;
        final TrackedHull hull;
        final List<TrackedPose> steps;
        int index;
        boolean stopRequested;
        boolean replanAfterStop;

        TrackedPoseRoute(Vec3 target, Vec3 safe, long generation, TrackedHull hull, List<TrackedPose> steps) {
            this.target = target;
            this.safe = safe;
            this.generation = generation;
            this.hull = hull;
            this.steps = List.copyOf(steps);
            this.index = Math.min(1, Math.max(0, this.steps.size() - 1));
        }

        boolean matches(Vec3 otherTarget) {
            return otherTarget != null && flatDistanceSqr(target, otherTarget) <= 4.0D;
        }

        TrackedHull hull() {
            return hull;
        }
    }

    private static final class TrackedRecovery {
        final Vec3 start;
        final Vec3 target;
        final TrackedHull hull;
        Vec3 reverseOrigin;
        long lastTick = Long.MIN_VALUE;
        int ticks;
        int reverseTicks;
        boolean brakingBeforeReverse = true;
        boolean brakingAfterReverse;

        TrackedRecovery(Vec3 start, Vec3 target, TrackedHull hull) {
            this.start = start;
            this.target = target;
            this.hull = hull;
            this.reverseOrigin = start;
        }
    }

    private static final class TrackedMotionWatch {
        Vec3 lastPosition;
        float lastYaw;
        long lastTick;
        int stalledTicks;

        TrackedMotionWatch(Vec3 lastPosition, float lastYaw, long lastTick) {
            this.lastPosition = lastPosition;
            this.lastYaw = lastYaw;
            this.lastTick = lastTick;
        }
    }

    private record TrackedHullPoint(Vector3f local, VehicleCubeOBB.CubeFace face) {}

    /** Immutable main-OBB geometry captured before the incremental pose search begins. */
    private static final class TrackedHull {
        final List<TrackedHullPoint> samples;
        final Vector3f centerLocal;
        final Quaternionf localObbRotation;
        final Vector3f extents;
        final double lowSideContactY;
        final double entityGroundOffset;

        TrackedHull(List<TrackedHullPoint> samples, Vector3f centerLocal, Quaternionf localObbRotation, Vector3f extents,
                    double lowSideContactY, double entityGroundOffset) {
            this.samples = samples;
            this.centerLocal = centerLocal;
            this.localObbRotation = localObbRotation;
            this.extents = extents;
            this.lowSideContactY = lowSideContactY;
            this.entityGroundOffset = entityGroundOffset;
        }

        static TrackedHull from(AbstractVehicle vehicle) {
            VehicleCubeOBB cube = vehicle.getMainCubeOBB();
            if (cube == null || cube.obb() == null || cube.cubePoints() == null || cube.cubePoints().isEmpty()) return null;
            Quaternionf vehicleRotation = new Quaternionf(vehicle.rotYXZ());
            Quaternionf inverse = new Quaternionf(vehicleRotation).invert();
            Quaternionf localRotation = new Quaternionf(inverse).mul(new Quaternionf(cube.obb().rotation()));
            Vec3 vehicleCenter = vehicle.position().add(vehicle.centerOffset == null ? Vec3.ZERO : vehicle.centerOffset);
            Vector3f centerLocal = new Vector3f(cube.obb().center()).sub(vehicleCenter.toVector3f());
            inverse.transform(centerLocal);
            List<TrackedHullPoint> samples = new ArrayList<>();
            for (VehicleCubeOBB.CubePoint point : cube.cubePoints()) {
                samples.add(new TrackedHullPoint(new Vector3f(point.obbLocalPos()), point.cubeFace()));
            }
            // PhysicsEngine ignores a vertical-face sample below this local height before
            // deciding whether it is a collision.  Capture it with the immutable pose hull so
            // planning and native driving agree even when the model has a non-default spaceY.
            double lowSideContactY = -cube.getHeight() / 2.0D + cube.spaceY;
            double entityGroundOffset = trackedEntityGroundOffset(vehicle, cube);
            return new TrackedHull(List.copyOf(samples), centerLocal, localRotation,
                    new Vector3f(cube.obb().extents()), lowSideContactY, entityGroundOffset);
        }

        private static double trackedEntityGroundOffset(AbstractVehicle vehicle, VehicleCubeOBB cube) {
            // This offset is a property of the model, not of the block currently under the
            // entity centre.  The old terrain scan returned 0.7-1.0 blocks while a long tank
            // bridged a slope or a single rock.  Every hypothetical pose was then raised by
            // that amount, its support plane was lowered by the same amount, and an otherwise
            // clear straight road became an "embedded" one-node search.
            //
            // Undo the live vehicle pitch/roll on every physical OBB vertex and measure the
            // canonical flat hull bottom relative to the entity origin.  Yaw cannot affect Y,
            // so this value stays invariant on slopes and uneven ground.
            Quaternionf inverseVehicleRotation = new Quaternionf(vehicle.rotYXZ()).invert();
            Vec3 centerOffset = vehicle.centerOffset == null ? Vec3.ZERO : vehicle.centerOffset;
            Vec3 vehicleCenter = vehicle.position().add(centerOffset);
            double flatBottomRelativeToEntity = Double.POSITIVE_INFINITY;
            for (Vector3f vertex : cube.obb().getVertices()) {
                Vector3f local = new Vector3f(vertex).sub(vehicleCenter.toVector3f());
                inverseVehicleRotation.transform(local);
                flatBottomRelativeToEntity = Math.min(flatBottomRelativeToEntity, centerOffset.y + local.y);
            }
            double offset = -flatBottomRelativeToEntity;
            return Double.isFinite(offset) ? offset : 0.0D;
        }

        TrackedPoseTransform transform(AbstractVehicle vehicle, Vec3 position, float yaw) {
            // Navigation is deliberately 2.5D.  A tracked hull's live pitch/roll describe its
            // *current* suspension contact with one particular slope; carrying those angles
            // into every hypothetical A* node makes a long tilted hull cut into otherwise flat
            // ground and disconnects the entire search graph.  Candidate Y plus the traversable
            // terrain band model grade changes; native YWZJ physics supplies the real pitch and
            // roll while the vehicle drives the accepted route.
            Quaternionf vehicleRotation = trackedPlanningRotation(yaw);
            Quaternionf obbRotation = new Quaternionf(vehicleRotation).mul(new Quaternionf(localObbRotation));
            Vector3f offset = new Vector3f(centerLocal);
            vehicleRotation.transform(offset);
            Vec3 centerBase = position.add(vehicle.centerOffset == null ? Vec3.ZERO : vehicle.centerOffset);
            Vector3f center = centerBase.add(offset.x, offset.y, offset.z).toVector3f();
            double minX = Double.POSITIVE_INFINITY, minY = Double.POSITIVE_INFINITY, minZ = Double.POSITIVE_INFINITY;
            double maxX = Double.NEGATIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY, maxZ = Double.NEGATIVE_INFINITY;
            for (TrackedHullPoint sample : samples) {
                Vector3f point = new Vector3f(sample.local);
                obbRotation.transform(point).add(center);
                minX = Math.min(minX, point.x); minY = Math.min(minY, point.y); minZ = Math.min(minZ, point.z);
                maxX = Math.max(maxX, point.x); maxY = Math.max(maxY, point.y); maxZ = Math.max(maxZ, point.z);
            }
            return new TrackedPoseTransform(this, vehicle.getUUID(), position.y, center, obbRotation,
                    new AABB(minX, minY, minZ, maxX, maxY, maxZ));
        }
    }

    static Quaternionf trackedPlanningRotation(float yaw) {
        return new Quaternionf().rotateY((float) Math.toRadians(-yaw));
    }

    private static final class TrackedPoseTransform {
        final TrackedHull hull;
        final UUID vehicleId;
        final double entityY;
        final Vector3f center;
        final Quaternionf rotation;
        final AABB bounds;

        TrackedPoseTransform(TrackedHull hull, UUID vehicleId, double entityY,
                             Vector3f center, Quaternionf rotation, AABB bounds) {
            this.hull = hull;
            this.vehicleId = vehicleId;
            this.entityY = entityY;
            this.center = center;
            this.rotation = rotation;
            this.bounds = bounds;
        }

        Vec3 world(Vector3f local) {
            Vector3f point = new Vector3f(local);
            rotation.transform(point).add(center);
            return new Vec3(point.x, point.y, point.z);
        }

        OBB navigationPrism() {
            // Native physics ignores vertical-face samples below lowSideContactY.  Trim that
            // track/curb region out of the planner prism instead of treating the ground as a
            // wall.  Small insets avoid classifying mathematical face-touching as penetration.
            float bottom = (float) Math.max(-hull.extents.y + 0.035D, hull.lowSideContactY + 0.035D);
            float top = hull.extents.y - 0.035F;
            if (top <= bottom) top = bottom + 0.05F;
            float localCenterY = (bottom + top) * 0.5F;
            Vector3f shiftedCenter = new Vector3f(0.0F, localCenterY, 0.0F);
            rotation.transform(shiftedCenter).add(center);
            Vector3f prismExtents = new Vector3f(
                    Math.max(0.05F, hull.extents.x - 0.035F),
                    Math.max(0.025F, (top - bottom) * 0.5F),
                    Math.max(0.05F, hull.extents.z - 0.035F));
            return new OBB(shiftedCenter, prismExtents, new Quaternionf(rotation));
        }

        AABB bounds() {
            return bounds;
        }
    }

    private record VehicleShape(double radius, double minYOffset, double maxYOffset, double voxelYOffset, int voxelHeight,
                                AABB nativeBox, Vec3 nativeBoxOrigin, boolean exactNativeBox) {
        static VehicleShape from(Entity vehicle) {
            AABB entityBox = vehicle.getBoundingBox();
            boolean wheeled = !isTrackedVehicle(vehicle);
            // TrackedVehicle's block physics uses getMainCubeOBB(), not its complete
            // selection/decorative OBB list.  The latter includes the turret, gun and
            // accessories and inflated a narrow road probe into a large square.
            AABB physicsObb = wheeled ? null : mainObbAabb(vehicle);
            AABB box = physicsObb == null ? entityBox : physicsObb;
            double radius = wheeled
                    ? Math.max(1.0D, Math.min(box.getXsize(), box.getZsize()) * 0.5D + 0.10D)
                    : Math.max(1.5D, Math.max(box.getXsize(), box.getZsize()) * 0.5D);
            double minYOffset = Math.min(-0.1D, box.minY - vehicle.getY());
            double maxYOffset = Math.max(1.2D, box.maxY - vehicle.getY() + 0.25D);
            double voxelYOffset = Math.max(0.0D, minYOffset + 0.18D);
            int voxelHeight = Math.max(2, Mth.ceil(maxYOffset - voxelYOffset + 0.15D));
            return new VehicleShape(radius, minYOffset, maxYOffset, voxelYOffset, voxelHeight,
                    box, vehicle.position(), true);
        }

        AABB aabbAt(Vec3 position) {
            if (exactNativeBox && nativeBox != null && nativeBoxOrigin != null) return nativeBox.move(position.subtract(nativeBoxOrigin));
            return new AABB(position.x - radius, position.y + minYOffset, position.z - radius,
                    position.x + radius, position.y + maxYOffset, position.z + radius);
        }
    }

    private record AvoidNode(int x, int z) {}

    private record PathState(AvoidNode node, double g, double h) {
        double f() {
            return g + h;
        }
    }

    private record Reservation(UUID owner, long expiresAt, double ownerSpeed) {}

    private static List<?> asList(Object object) {
        return object instanceof List<?> list ? list : null;
    }

    private static Object invokeNoArg(Object target, String name) {
        return invoke(target, name, new Class<?>[0]);
    }

    private static Object invoke(Object target, String name, Class<?>[] parameterTypes, Object... args) {
        if (target == null) return null;
        MethodKey key = MethodKey.of(target.getClass(), name, parameterTypes);
        if (BROKEN_METHODS.contains(key)) return null;
        try {
            Method method = findMethod(target.getClass(), name, parameterTypes);
            if (method == null) return null;
            return method.invoke(target, args);
        } catch (ReflectiveOperationException | RuntimeException exception) {
            tripBrokenMethod(key, exception);
            return null;
        }
    }

    private static void tripBrokenMethod(MethodKey key, Exception exception) {
        if (BROKEN_METHODS.add(key)) {
            Throwable cause = exception instanceof java.lang.reflect.InvocationTargetException invocation && invocation.getCause() != null ? invocation.getCause() : exception;
            LOGGER.error("YWZJ vehicle reflective bridge method {}#{} failed and has been disabled.", key.type().getName(), key.name(), cause);
        }
    }

    private static Method findMethod(Class<?> type, String name, Class<?>... parameterTypes) {
        if (type == null || name == null) return null;
        return METHOD_CACHE.computeIfAbsent(MethodKey.of(type, name, parameterTypes), key -> Optional.ofNullable(findMethodUncached(key.type(), key.name(), key.parameterTypes().toArray(Class<?>[]::new)))).orElse(null);
    }

    private static Method findMethodUncached(Class<?> type, String name, Class<?>... parameterTypes) {
        for (Class<?> cursor = type; cursor != null; cursor = cursor.getSuperclass()) {
            try {
                Method method = cursor.getDeclaredMethod(name, parameterTypes);
                method.setAccessible(true);
                return method;
            } catch (NoSuchMethodException ignored) {
            }
        }
        return null;
    }

    private static Object readMember(Object target, String name) {
        if (target == null) return null;
        try {
            Field field = findField(target.getClass(), name);
            if (field == null) return null;
            return field.get(target);
        } catch (ReflectiveOperationException | RuntimeException ignored) {
            return null;
        }
    }

    private static boolean writeMember(Object target, String name, Object value) {
        if (target == null) return false;
        try {
            Field field = findField(target.getClass(), name);
            if (field == null) return false;
            field.set(target, value);
            return true;
        } catch (ReflectiveOperationException | RuntimeException ignored) {
            return false;
        }
    }

    private static Field findField(Class<?> type, String name) {
        if (type == null || name == null) return null;
        return FIELD_CACHE.computeIfAbsent(new FieldKey(type, name), key -> Optional.ofNullable(findFieldUncached(key.type(), key.name()))).orElse(null);
    }

    private static Field findFieldUncached(Class<?> type, String name) {
        for (Class<?> cursor = type; cursor != null; cursor = cursor.getSuperclass()) {
            try {
                Field field = cursor.getDeclaredField(name);
                field.setAccessible(true);
                return field;
            } catch (NoSuchFieldException ignored) {
            }
        }
        return null;
    }

    /**
     * The released YWZJ renderer unconditionally draws every special bone after
     * the base model, including MUZZLE_FLASH. Temporarily remove only those entries
     * from the shared display model while Dominion Sword renders an off-screen
     * portrait, then restore the exact original list and order.
     */
    private static final class MuzzleFlashPortraitScope {
        @SuppressWarnings("unchecked")
        private static PortraitRenderScope suppress(Entity vehicle) {
            try {
                Class<?> assetsType = Class.forName("org.ywzj.vehicle.client.resource.ClientAssetsManager");
                Object manager = assetsType.getField("INSTANCE").get(null);
                Object displayOptional = invoke(manager, "getVehicleDisplay", new Class<?>[]{net.minecraft.resources.ResourceLocation.class}, ((AbstractVehicle) vehicle).getDisplayId());
                Object display = displayOptional instanceof Optional<?> optional ? optional.orElse(null) : displayOptional;
                Object model = invoke(display, "getModel", new Class<?>[0]);
                if (model == null) return PortraitRenderScope.NOOP;
                Object entriesValue = readMember(model, "specialBoneEntries");
                if (!(entriesValue instanceof List<?>)) entriesValue = readMember(model, "bakedSpecialBoneEntries");
                if (!(entriesValue instanceof List<?> entries) || entries.isEmpty()) return PortraitRenderScope.NOOP;
                List<Object> mutable = (List<Object>) entries;
                List<Object> original = new ArrayList<>(mutable);
                List<BoneVisibility> hiddenBones = new ArrayList<>();
                for (Object entry : original) {
                    if (isMuzzleFlashEntry(entry)) {
                        BoneVisibility visibility = hideMuzzleBone(model, entry);
                        if (visibility != null) hiddenBones.add(visibility);
                    }
                }
                try {
                    mutable.removeIf(MuzzleFlashPortraitScope::isMuzzleFlashEntry);
                } catch (RuntimeException exception) {
                    restoreBoneVisibility(hiddenBones);
                    return PortraitRenderScope.NOOP;
                }
                if (mutable.size() == original.size()) {
                    restoreBoneVisibility(hiddenBones);
                    return PortraitRenderScope.NOOP;
                }
                return () -> {
                    try {
                        mutable.clear();
                        mutable.addAll(original);
                    } catch (RuntimeException ignored) {
                    } finally {
                        restoreBoneVisibility(hiddenBones);
                    }
                };
            } catch (ReflectiveOperationException | RuntimeException | LinkageError ignored) {
                return PortraitRenderScope.NOOP;
            }
        }

        private static boolean isMuzzleFlashEntry(Object entry) {
            Object effect = readMember(entry, "effect");
            if (effect == null) effect = invoke(entry, "effect", new Class<?>[0]);
            Object type = readMember(effect, "type");
            return type instanceof Enum<?> value && "MUZZLE_FLASH".equals(value.name());
        }

        private static BoneVisibility hideMuzzleBone(Object model, Object entry) {
            try {
                // Released 0.5.6 stores the BedrockBone directly on the entry.
                Object bone = readMember(entry, "bone");
                // Newer source revisions store an index into the baked instance.
                if (bone == null) {
                    Object index = invoke(entry, "boneIndex", new Class<?>[0]);
                    Object instance = invoke(model, "getDefaultModelInstance", new Class<?>[0]);
                    if (index instanceof Number number && instance != null)
                        bone = invoke(instance, "getBone", new Class<?>[]{int.class}, number.intValue());
                }
                if (bone == null) return null;
                Field visible = findField(bone.getClass(), "visible");
                if (visible == null || visible.getType() != boolean.class) return null;
                boolean previous = visible.getBoolean(bone);
                visible.setBoolean(bone, false);
                return new BoneVisibility(bone, visible, previous);
            } catch (ReflectiveOperationException | RuntimeException ignored) {
                return null;
            }
        }

        private static void restoreBoneVisibility(List<BoneVisibility> hiddenBones) {
            for (BoneVisibility visibility : hiddenBones) visibility.restore();
        }

        private record BoneVisibility(Object bone, Field visible, boolean previous) {
            private void restore() {
                try { visible.setBoolean(bone, previous); }
                catch (ReflectiveOperationException | RuntimeException ignored) {}
            }
        }
    }

    private record ClassNameKey(Class<?> type, String name) {}
    private record FieldKey(Class<?> type, String name) {}
    private record MethodKey(Class<?> type, String name, List<Class<?>> parameterTypes) {
        private static MethodKey of(Class<?> type, String name, Class<?>... parameterTypes) {
            return new MethodKey(type, name, List.of(parameterTypes));
        }
    }

    private static int readInt(Object value, int fallback) {
        return value instanceof Number number ? number.intValue() : fallback;
    }

    private static float readFloat(Object value, float fallback) {
        return value instanceof Number number ? number.floatValue() : fallback;
    }

    private static double readDouble(Object value, double fallback) {
        return value instanceof Number number ? number.doubleValue() : fallback;
    }
}
