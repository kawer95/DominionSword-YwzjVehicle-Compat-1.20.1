package com.arxyt.dominionsword.ywzjvehiclecompat;

import com.arxyt.dominionsword.api.DominionVehicleAdapters;
import net.minecraftforge.common.MinecraftForge;
import net.minecraftforge.event.TickEvent;
import net.minecraftforge.event.entity.EntityJoinLevelEvent;
import net.minecraftforge.event.entity.EntityLeaveLevelEvent;
import net.minecraftforge.fml.ModLoadingContext;
import net.minecraftforge.fml.common.Mod;
import net.minecraftforge.fml.config.ModConfig;

@Mod(DominionSwordYwzjVehicleCompatMod.MODID)
public final class DominionSwordYwzjVehicleCompatMod {
    public static final String MODID = "dominionsword_ywzjvehicle_compat";
    private final YwzjVehicleAdapter vehicleAdapter = new YwzjVehicleAdapter();

    public DominionSwordYwzjVehicleCompatMod() {
        ModLoadingContext.get().registerConfig(ModConfig.Type.COMMON, YwzjVehicleCompatConfig.SPEC);
        DominionVehicleAdapters.register(vehicleAdapter);
        MinecraftForge.EVENT_BUS.addListener(this::onServerTick);
        MinecraftForge.EVENT_BUS.addListener(this::onEntityJoin);
        MinecraftForge.EVENT_BUS.addListener(this::onEntityLeave);
    }

    private void onServerTick(TickEvent.ServerTickEvent event) {
        if (event.phase == TickEvent.Phase.START) {
            // Ground vehicles consume input during their own physical tick.  The
            // planner runs at a lower frequency, so replay the latest autonomous
            // command before that physics step.
            vehicleAdapter.tickGroundControl(event.getServer());
        } else {
            vehicleAdapter.tickHelicopterAutopilot(event.getServer());
        }
    }

    private void onEntityJoin(EntityJoinLevelEvent event) {
        if (!event.getLevel().isClientSide()) vehicleAdapter.onEntityLoaded(event.getEntity());
    }

    private void onEntityLeave(EntityLeaveLevelEvent event) {
        if (!event.getLevel().isClientSide()) vehicleAdapter.onEntityUnloaded(event.getEntity());
    }
}
