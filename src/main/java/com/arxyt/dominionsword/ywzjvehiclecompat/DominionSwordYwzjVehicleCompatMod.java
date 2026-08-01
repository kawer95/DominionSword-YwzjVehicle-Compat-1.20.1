package com.arxyt.dominionsword.ywzjvehiclecompat;

import com.arxyt.dominionsword.api.DominionVehicleAdapters;
import net.minecraftforge.common.MinecraftForge;
import net.minecraftforge.event.TickEvent;
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
    }

    private void onServerTick(TickEvent.ServerTickEvent event) {
        if (event.phase != TickEvent.Phase.END) return;
        vehicleAdapter.tickHelicopterAutopilot(event.getServer());
    }
}
