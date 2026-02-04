package frc.robot.subsystems.stats;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.nio.ByteBuffer;
import org.littletonrobotics.junction.Logger;

public class StatsSubsystem extends SubsystemBase {
    Stats stats = new Stats();

    @Override
    public void periodic() {
        stats.totalTravelDist += 1;
        Logger.recordOutput("Stats", stats);
    }

    public static class Stats implements StructSerializable {
        double totalTravelDist;
        double totalTravelTime;
        double totalBallShots;
        double totalButtonsPressed;
        double totalDistanceClimbed;
        double totalClimbs;
        double totalTurretRotations;

        public Stats(
                double totalTravelDist,
                double totalTravelTime,
                double totalBallShots,
                double totalButtonsPressed,
                double totalDistanceClimbed,
                double totalClimbs,
                double totalTurretRotations) {
            this.totalTravelDist = totalTravelDist;
            this.totalTravelTime = totalTravelTime;
            this.totalBallShots = totalBallShots;
            this.totalButtonsPressed = totalButtonsPressed;
            this.totalDistanceClimbed = totalDistanceClimbed;
            this.totalClimbs = totalClimbs;
            this.totalTurretRotations = totalTurretRotations;
        }

        public Stats() {}

        public static final Struct<Stats> struct = new StatsStruct();
    }

    public static class StatsStruct implements Struct<Stats> {
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
            return new Stats(
                    bb.getDouble(),
                    bb.getDouble(),
                    bb.getDouble(),
                    bb.getDouble(),
                    bb.getDouble(),
                    bb.getDouble(),
                    bb.getDouble());
        }

        @Override
        public void pack(ByteBuffer bb, Stats value) {
            bb.putDouble(value.totalTravelDist);
            bb.putDouble(value.totalTravelTime);
            bb.putDouble(value.totalBallShots);
            bb.putDouble(value.totalButtonsPressed);
            bb.putDouble(value.totalDistanceClimbed);
            bb.putDouble(value.totalClimbs);
            bb.putDouble(value.totalTurretRotations);
        }

        @Override
        public boolean isImmutable() {
            return false;
        }
    }
}
