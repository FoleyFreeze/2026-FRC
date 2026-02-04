package frc.robot.subsystems.stats;

import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class StatsStruct implements Struct<Stats> {
    @Override
    public Class<Stats> getTypeClass() {
        return Stats.class;
    }

    @Override
    public String getTypeName() {
        return "Stats";
    }

    @Override
    public int getSize() {
        return kSizeDouble * 7;
    }

    @Override
    public String getSchema() {
        return "double totalTravelDist;double totalTravelTime;"
                + "double totalBallShots;double totalButtonsPressed;"
                + "double totalDistanceClimbed;double totalClimbs;double totalTurretRotations";
    }

    @Override
    public Stats unpack(ByteBuffer bb) {
        return new Stats();
        // return new Stats(
        //         bb.getDouble(),
        //         bb.getDouble(),
        //         bb.getDouble(),
        //         bb.getDouble(),
        //         bb.getDouble(),
        //         bb.getDouble(),
        //         bb.getDouble());
    }

    @Override
    public void pack(ByteBuffer bb, Stats value) {
        // bb.putDouble(value.totalTravelDist);
        // bb.putDouble(value.totalTravelTime);
        // bb.putDouble(value.totalBallShots);
        // bb.putDouble(value.totalButtonsPressed);
        // bb.putDouble(value.totalDistanceClimbed);
        // bb.putDouble(value.totalClimbs);
        // bb.putDouble(value.totalTurretRotations);
    }

    @Override
    public boolean isImmutable() {
        return false;
    }
}
