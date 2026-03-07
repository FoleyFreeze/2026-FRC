package frc.robot.subsystems.stats;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.RobotContainer;

public class Stats implements StructSerializable {
    RobotContainer r;

    double totalTravelDist;
    double totalDistFLwheel;
    double totalDistFRwheel;
    double totalDistRLwheel;
    double totalDistRRwheel;
    double totalTravelTime;

    double totalBallShots;
    double totalTurretRotations;
    double totalTurretFlips;
    double totalHubShootTime;
    double totalPassShootTime;
    
    double totalButtonsPressed;
    
    double totalDistanceClimbed;
    double totalClimbs;
    
    double totalEnabledTagImages;
    double totalEnabledBallImages;

    public Stats(
            double totalTravelDist,
            double totalDistFLwheel,
            double totalDistFRwheel,
            double totalDistRLwheel,
            double totalDistRRwheel,
            double totalTravelTime,
            double totalBallShots,
            double totalTurretRotations,
            double totalTurretFlips,
            double totalHubShootTime,
            double totalPassShootTime,
            double totalButtonsPressed,
            double totalDistanceClimbed,
            double totalClimbs,
            double totalEnabledTagImages,
            double totalEnabledBallImages) {
        this.totalTravelDist = totalTravelDist;
        this.totalDistFLwheel = totalDistFLwheel;
        this.totalDistFRwheel = totalDistFRwheel;
        this.totalDistRLwheel = totalDistRLwheel;
        this.totalDistRRwheel = totalDistRRwheel;
        this.totalTravelTime = totalTravelTime;
        this.totalBallShots = totalBallShots;
        this.totalTurretRotations = totalTurretRotations;
        this.totalTurretFlips = totalTurretFlips;
        this.totalHubShootTime = totalHubShootTime;
        this.totalPassShootTime = totalPassShootTime;
        this.totalButtonsPressed = totalButtonsPressed;
        this.totalDistanceClimbed = totalDistanceClimbed;
        this.totalClimbs = totalClimbs;
        this.totalEnabledTagImages = totalEnabledTagImages;
        this.totalEnabledBallImages = totalEnabledBallImages;
    }

    public Stats() {}

    // public static final Struct<Stats> struct = new StatsStruct();
    public static final Struct<Stats> struct = new StatsStructReflection();
}
