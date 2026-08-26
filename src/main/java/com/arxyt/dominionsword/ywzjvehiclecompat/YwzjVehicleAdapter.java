package com.arxyt.dominionsword.ywzjvehiclecompat;

import com.arxyt.dominionsword.api.DominionControlApi;
import com.arxyt.dominionsword.api.DominionVehicleAdapter;
import com.arxyt.dominionsword.api.DominionAsyncGridPlanner;
import com.arxyt.dominionsword.control.PlayerControl;
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
import org.ywzj.vehicle.vehicle.control.ControlUnit;
import org.ywzj.vehicle.vehicle.part.WeaponUnit;
import org.ywzj.vehicle.vehicle.pojo.AimContext;
import org.ywzj.vehicle.vehicle.weapon.AbstractVehicleWeapon;

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
    /** Ground tasks need a server-side input pulse before the native vehicle tick. */
    private static final Map<ResourceKey<Level>, Set<UUID>> ACTIVE_GROUND_AUTOPILOTS = new ConcurrentHashMap<>();
    private static final Map<UUID, GroundControlPulse> GROUND_CONTROL_PULSES = new ConcurrentHashMap<>();
    private static final String OFFLINE_VEHICLE_MOVE = "DominionOfflineVehicleMove";
    private static final String OFFLINE_VEHICLE_X = "DominionOfflineVehicleX";
    private static final String OFFLINE_VEHICLE_Y = "DominionOfflineVehicleY";
    private static final String OFFLINE_VEHICLE_Z = "DominionOfflineVehicleZ";
    private static final String VEHICLE_CONTROLLER = "dominionsword_controller_player";
    private static final String PLAYER_SELECTIONS = "DominionSelected";
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
    private static final Map<UUID, Long> HELICOPTER_SELECTION_TRACE_TICKS = new ConcurrentHashMap<>();
    private static final int ASYNC_SNAPSHOT_CELLS_PER_TICK = 72;
    private static final String PATH_ASYNC_PENDING = "DominionSwordYwzjPathAsyncPending";
    private static final String PATH_DEBUG_PHASE = "DominionSwordYwzjPathDebugPhase";
    private static final String PATH_DEBUG_TICK = "DominionSwordYwzjPathDebugTick";

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
            registerGroundAutopilot(vehicle);
            ((AbstractVehicle) vehicle).toggleEngine(Boolean.TRUE);
            LagTrace.mark("engine");

            VehicleShape shape = VehicleShape.from(vehicle);
            LagTrace.mark("shape");
            ensureRoute(player, vehicle, target, shape);
            LagTrace.mark("ensure_route");
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
            boolean forward = !brake && !closeBehind && absYaw < 115.0D;
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
            if (!forward && !backward && !brake && absYaw >= TURN_IN_PLACE_ANGLE) {
                // Tracked vehicles rotate in place; wheeled vehicles with a large turn radius back toward
                // the target instead of a forward crawl (which barely turns them at large turn radii).
                if (!isTrackedVehicle(vehicle)) {
                    backward = true;
                    reverseSteer = true;
                }
            }

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
     * Runs before level entities tick. The native tracked-vehicle implementation consumes
     * ControlUnit flags during its own tick, so writing the flags only at ServerTick.END
     * can devolve into a single command pulse when the RTS selection changes.
     */
    public void tickGroundAutopilot(net.minecraft.server.MinecraftServer server) {
        if (server == null) return;
        reapplyGroundControlPulses(server);
        ACTIVE_GROUND_AUTOPILOTS.entrySet().removeIf(entry -> {
            ServerLevel level = server.getLevel(entry.getKey());
            if (level == null) return true;
            entry.getValue().removeIf(id -> advanceUnselectedGroundTask(server, level, id));
            return entry.getValue().isEmpty();
        });
    }

    public void onEntityLoaded(Entity entity) {
        if (supports(entity) && !isRotaryWingVehicle(entity)) registerGroundAutopilot(entity);
        Entity vehicle = isRotaryWingVehicle(entity) ? entity : entity instanceof Mob mob ? mob.getVehicle() : null;
        if (vehicle == null || !isRotaryWingVehicle(vehicle) || !(driver(vehicle) instanceof Mob pilot)) return;
        String mode = pilot.getPersistentData().getString(HELI_MODE);
        if (!mode.isBlank() && !"LANDED".equals(mode)) registerActiveHelicopter(vehicle);
    }

    public void onEntityUnloaded(Entity entity) {
        if (supports(entity) && !isRotaryWingVehicle(entity)) {
            unregisterGroundAutopilot(entity);
            GROUND_CONTROL_PULSES.remove(entity.getUUID());
        }
        Entity vehicle = isRotaryWingVehicle(entity) ? entity : entity instanceof Mob mob ? mob.getVehicle() : null;
        if (vehicle != null) unregisterActiveHelicopter(vehicle);
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

    private static void registerGroundAutopilot(Entity vehicle) {
        if (vehicle == null || vehicle.level().isClientSide() || isRotaryWingVehicle(vehicle)
                || !vehicle.getPersistentData().getBoolean(OFFLINE_VEHICLE_MOVE)) return;
        ACTIVE_GROUND_AUTOPILOTS.computeIfAbsent(vehicle.level().dimension(), ignored -> ConcurrentHashMap.newKeySet())
                .add(vehicle.getUUID());
    }

    private static void unregisterGroundAutopilot(Entity vehicle) {
        if (vehicle == null) return;
        Set<UUID> entries = ACTIVE_GROUND_AUTOPILOTS.get(vehicle.level().dimension());
        if (entries == null) return;
        entries.remove(vehicle.getUUID());
        if (entries.isEmpty()) ACTIVE_GROUND_AUTOPILOTS.remove(vehicle.level().dimension(), entries);
    }

    /** A selected vehicle is still driven by Dominion Sword at ServerTick.END. */
    private static boolean selectedByCurrentController(net.minecraft.server.MinecraftServer server, Entity vehicle) {
        CompoundTag task = vehicle.getPersistentData();
        if (!task.hasUUID(VEHICLE_CONTROLLER)) return false;
        ServerPlayer controller = server.getPlayerList().getPlayer(task.getUUID(VEHICLE_CONTROLLER));
        if (controller == null) return false;
        ListTag selected = controller.getPersistentData().getList(PLAYER_SELECTIONS, Tag.TAG_COMPOUND);
        for (int i = 0; i < selected.size(); i++) {
            CompoundTag entry = selected.getCompound(i);
            if (entry.hasUUID("id") && vehicle.getUUID().equals(entry.getUUID("id"))) return true;
        }
        return false;
    }

    private boolean advanceUnselectedGroundTask(net.minecraft.server.MinecraftServer server, ServerLevel level, UUID id) {
        Entity vehicle = level.getEntity(id);
        if (!supports(vehicle) || isRotaryWingVehicle(vehicle) || !vehicle.isAlive()) return true;
        CompoundTag task = vehicle.getPersistentData();
        if (!task.getBoolean(OFFLINE_VEHICLE_MOVE)) {
            GROUND_CONTROL_PULSES.remove(id);
            return true;
        }
        // Never overwrite a real player's manual input. The player can dismount and the
        // maid pilot will resume the persistent route on the following server tick.
        if (!(driver(vehicle) instanceof Mob)) {
            GROUND_CONTROL_PULSES.remove(id);
            return false;
        }
        if (selectedByCurrentController(server, vehicle)) return false;
        Vec3 target = new Vec3(task.getDouble(OFFLINE_VEHICLE_X), task.getDouble(OFFLINE_VEHICLE_Y), task.getDouble(OFFLINE_VEHICLE_Z));
        VehicleShape shape = VehicleShape.from(vehicle);
        Vec3 safeTarget = activeSafeTarget(vehicle, target);
        if (hasCapturedFinalTarget(vehicle, safeTarget, shape) || flatDistance(vehicle.position(), safeTarget) <= ARRIVE_RADIUS) {
            stopVehicle(vehicle);
            PlayerControl.completeRedirectedVehicleMove(vehicle);
            return true;
        }
        move(null, vehicle, target);
        return false;
    }

    private static void reapplyGroundControlPulses(net.minecraft.server.MinecraftServer server) {
        GROUND_CONTROL_PULSES.entrySet().removeIf(entry -> {
            GroundControlPulse pulse = entry.getValue();
            ServerLevel level = server.getLevel(pulse.dimension());
            if (level == null) return true;
            Entity entity = level.getEntity(entry.getKey());
            if (!(entity instanceof AbstractVehicle vehicle) || !entity.isAlive()
                    || !entity.getPersistentData().getBoolean(OFFLINE_VEHICLE_MOVE)) return true;
            LivingEntity operator = vehicle.controlUnit.getOperator();
            if (!(operator instanceof Mob) || operator.getVehicle() != vehicle || !pulse.driverId().equals(operator.getUUID())) return true;
            vehicle.toggleEngine(Boolean.TRUE);
            setControl(vehicle.controlUnit, pulse.forward(), pulse.backward(), pulse.right(), pulse.left(), pulse.brake());
            return false;
        });
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
        CompoundTag data = vehicle.getPersistentData();
        if (data.getBoolean(PATH_ASYNC_PENDING)) {
            advanceAsyncRoute(player, vehicle, target, ignoredVehicles);
            return;
        }
        if (hasActiveRoute(vehicle, target)
                && vehicle.getPersistentData().contains(PATH_POINTS, Tag.TAG_LIST)
                && !vehicle.getPersistentData().getBoolean(PATH_BLOCKED)) return;
        VehicleShape shape = VehicleShape.from(vehicle);
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

    private static Vec3 navigationTarget(ServerPlayer player, Entity vehicle, Vec3 finalTarget, VehicleShape shape) {
        if (vehicle.getPersistentData().getBoolean(PATH_ASYNC_PENDING)) return null;
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

    private static boolean startAsyncRoute(Entity vehicle, Vec3 safe, VehicleShape shape, Set<UUID> ignored, long generation) {
        if (ASYNC_ROUTES.containsKey(vehicle.getUUID())) return true;
        int radius = Math.min(32, (int) Math.ceil(PATH_SEARCH_RADIUS / PATH_STEP));
        Vec3 start = vehicle.position();
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
        if (flatDistance(route.get(route.size() - 1), build.safe) > 1.0D) route.add(build.safe);
        storeRoute(vehicle, build.safe, route, firstUsefulIndex(route, vehicle.position(), build.shape));
        refreshPlannedPath(player, vehicle, route);
        pathDebug(vehicle, "ASYNC_ROUTE_APPLIED", "generation=%d cells=%d visited=%d points=%d", build.generation, build.cells.size(), result.visited(), route.size());
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
            return routeLookaheadPoint(points, index, position, finalTarget, shape, speed);
        }
        clearRoute(vehicle);
        return finalTarget;
    }

    private static Vec3 routeLookaheadPoint(ListTag points, int index, Vec3 position, Vec3 finalTarget, VehicleShape shape, double speed) {
        double lookahead = dynamicLookahead(speed, shape);
        Vec3 previous = position;
        double walked = 0.0D;
        for (int i = Math.max(1, index); i < points.size(); i++) {
            Vec3 point = readPathPoint(points.getCompound(i));
            if (point == null) break;
            walked += flatDistance(previous, point);
            if (walked >= lookahead) return point;
            previous = point;
        }
        return finalTarget;
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
        if (direct != null) return direct;
        Vec3 origin = vehicle.position();
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
        SafeCandidate best = candidates.get(0);
        int checkLimit = Math.min(5, candidates.size());
        for (int i = 0; i < checkLimit; i++) {
            SafeCandidate candidate = candidates.get(i);
            if (canTravelDirect(vehicle, origin, candidate.position(), shape, Math.max(PATH_LOOKAHEAD_MIN, flatDistance(origin, candidate.position()) + shape.radius()))) {
                double score = candidate.score() - 4.0D;
                if (score < best.score()) best = new SafeCandidate(candidate.position(), score);
                break;
            }
        }
        return best.position();
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
        return PATH_STEP * MAX_TRAVEL_STEP_HEIGHT;
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
        int radius = Math.max(1, (int) Math.ceil(shape.radius()));
        int height = Math.max(2, shape.voxelHeight());
        BlockPos center = BlockPos.containing(position.x, position.y + shape.voxelYOffset(), position.z);
        boolean supported = false;
        for (int dx = -radius; dx <= radius; dx++) {
            for (int dz = -radius; dz <= radius; dz++) {
                BlockPos floor = center.offset(dx, -1, dz);
                if (!level.getBlockState(floor).getCollisionShape(level, floor).isEmpty()) supported = true;
                for (int dy = 0; dy < height; dy++) {
                    BlockPos pos = center.offset(dx, dy, dz);
                    BlockState state = level.getBlockState(pos);
                    if (!state.getCollisionShape(level, pos).isEmpty()) return false;
                }
            }
        }
        return supported;
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
        boolean cannotArc = !tracked && !shortReverseTarget && !threePointCooldown && cannotArcToTarget(vehicle, absYaw, distance, shape);
        double turnPenalty = absYaw / 8.0D;
        double stopPenalty = Math.max(0.0D, speed) * 18.0D;
        double forwardEta = distance / 0.10D + turnPenalty + stopPenalty;
        if (cannotArc) forwardEta += 80.0D;
        double reverseEta = distance / 0.06D + Math.abs(Mth.wrapDegrees(yawDiff - 180.0F)) / 8.0D + stopPenalty * 1.15D;
        double turnAroundEta = stopPenalty + 50.0D + distance / 0.10D;
        DriveMode mode;
        if (shortReverseTarget) {
            mode = DriveMode.REVERSE_SHORT;
        } else if (cannotArc) {
            mode = DriveMode.THREE_POINT;
        } else if (!tracked && rearTarget && distance > CLOSE_REVERSE_RADIUS) {
            mode = DriveMode.TURN_AROUND;
        } else {
            mode = DriveMode.FORWARD;
        }
        return new DriveDecision(mode, forwardEta, reverseEta, turnAroundEta);
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
        ControlUnit control = ywzjVehicle.controlUnit;
        setControl(control, forward, backward, right, left, brake);
        LivingEntity operator = control.getOperator();
        if (operator instanceof Mob && operator.getVehicle() == vehicle && !vehicle.level().isClientSide()) {
            GROUND_CONTROL_PULSES.put(vehicle.getUUID(), new GroundControlPulse(vehicle.level().dimension(), operator.getUUID(),
                    forward, backward, right, left, brake));
        }
        alignMainWeaponsForward(vehicle, forward);
    }

    private static void setControl(ControlUnit control, boolean forward, boolean backward, boolean right, boolean left, boolean brake) {
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

    private static void alignMainWeaponsForward(Entity vehicle, boolean moving) {
        if (!moving) return;
        Vec3 front = vehicle.position().add(vehicle.getLookAngle().multiply(24.0D, 0.0D, 24.0D));
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
        if (vehicle instanceof AbstractVehicle ywzjVehicle) {
            ywzjVehicle.controlUnit.reset();
            GROUND_CONTROL_PULSES.remove(vehicle.getUUID());
        }
    }

    private static void pathDebug(Entity vehicle, String phase, String format, Object... args) {
        try {
            if (vehicle != null && !vehicle.level().isClientSide()) {
                CompoundTag data = vehicle.getPersistentData();
                long now = vehicle.level().getGameTime();
                if (phase.equals(data.getString(PATH_DEBUG_PHASE)) && now - data.getLong(PATH_DEBUG_TICK) < 20L) return;
                data.putString(PATH_DEBUG_PHASE, phase);
                data.putLong(PATH_DEBUG_TICK, now);
            }
            String message = args == null || args.length == 0 ? format : String.format(Locale.ROOT, format, args);
            System.out.println(String.format(Locale.ROOT, "[DS-YWZJ-PATH] phase=%s vehicle=%s pos=%s yaw=%.1f speed=%.3f %s",
                    phase,
                    vehicle == null ? "null" : vehicle.getStringUUID(),
                    vehicle == null ? "null" : fmt(vehicle.position()),
                    vehicle == null ? 0.0F : vehicle.getYRot(),
                    vehicle == null ? 0.0D : horizontalSpeed(vehicle),
                    message));
        } catch (RuntimeException ignored) {
        }
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

    private record VehicleShape(double radius, double minYOffset, double maxYOffset, double voxelYOffset, int voxelHeight) {
        static VehicleShape from(Entity vehicle) {
            AABB obb = allObbAabb(vehicle);
            AABB box = obb == null ? vehicle.getBoundingBox() : union(obb, vehicle.getBoundingBox());
            double radius = Math.max(1.5D, Math.max(box.getXsize(), box.getZsize()) * 0.5D + 1.0D);
            double minYOffset = Math.min(-0.1D, box.minY - vehicle.getY());
            double maxYOffset = Math.max(1.2D, box.maxY - vehicle.getY() + 0.25D);
            double voxelYOffset = Math.max(0.0D, minYOffset + 0.18D);
            int voxelHeight = Math.max(2, Mth.ceil(maxYOffset - voxelYOffset + 0.15D));
            return new VehicleShape(radius, minYOffset, maxYOffset, voxelYOffset, voxelHeight);
        }

        AABB aabbAt(Vec3 position) {
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
    private record GroundControlPulse(ResourceKey<Level> dimension, UUID driverId, boolean forward, boolean backward,
                                      boolean right, boolean left, boolean brake) {}
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
