package frc.robot.subsystems.stats;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.RobotContainer;

public class Stats implements StructSerializable {
    RobotContainer r;

    double totalTravelDist;
    double totalTravelTime;
    double totalBallShots;
    double totalButtonsPressed;
    double totalDistanceClimbed;
    double totalClimbs;
    double totalTurretRotations;
    double value3;

    public Stats(
            double totalTravelDist,
            double totalTravelTime,
            double totalBallShots,
            double totalButtonsPressed,
            double totalDistanceClimbed,
            double totalClimbs,
            double totalTurretRotations,
            double value3) {
        this.totalTravelDist = totalTravelDist;
        this.totalTravelTime = totalTravelTime;
        this.totalBallShots = totalBallShots;
        this.totalButtonsPressed = totalButtonsPressed;
        this.totalDistanceClimbed = totalDistanceClimbed;
        this.totalClimbs = totalClimbs;
        this.totalTurretRotations = totalTurretRotations;
        this.value3 = value3;
    }

    public Stats() {}

    // public static final Struct<Stats> struct = new StatsStruct();
    public static final Struct<Stats> struct = new StatsStructReflection();
}
