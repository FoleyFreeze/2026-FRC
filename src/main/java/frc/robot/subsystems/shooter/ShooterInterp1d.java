package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;

public class ShooterInterp1d {
    public static record DataPoint(double rpm, double angle, double hood, double time) {}

    private static final double[] distAxis = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    };

    private static final double[] rpmTable = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    private static final double[] hoodAngleTable = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    private static final double[] timeTable = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    public ShooterInterp1d() {}

    public DataPoint get(double dist) {
        return get(dist, 0);
    }

    public DataPoint get(double dist, double angle) {
        int distIdx = 1;
        while (distIdx < distAxis.length - 1 && dist > distAxis[distIdx]) {
            distIdx++;
        }
        int pDistIdx = distIdx - 1;

        double fracX = MathUtil.inverseInterpolate(distAxis[pDistIdx], distAxis[distIdx], dist);

        double rpm = MathUtil.interpolate(rpmTable[pDistIdx], rpmTable[distIdx], fracX);
        double hood =
                MathUtil.interpolate(hoodAngleTable[pDistIdx], hoodAngleTable[distIdx], fracX);
        double time = MathUtil.interpolate(timeTable[pDistIdx], timeTable[distIdx], fracX);

        return new DataPoint(rpm, angle, hood, time);
    }

    public double getTime(double dist) {
        int distIdx = 1;
        while (distIdx < distAxis.length - 1 && dist > distAxis[distIdx]) {
            distIdx++;
        }
        int pDistIdx = distIdx - 1;

        double fracX = MathUtil.inverseInterpolate(distAxis[pDistIdx], distAxis[distIdx], dist);

        double time = MathUtil.interpolate(timeTable[pDistIdx], timeTable[distIdx], fracX);

        return time;
    }

    private static final int reps = 3;

    public DataPoint get(Translation2d goal, Translation2d pos, ChassisSpeeds vel) {
        // rep 1
        double dist = goal.minus(pos).getNorm();
        double time = getTime(dist);
        // rep 2 - x
        for (int i = 1; i < reps - 1; i++) {
            Translation2d offset =
                    new Translation2d(-vel.vxMetersPerSecond * time, -vel.vyMetersPerSecond * time);
            dist = goal.plus(offset).minus(pos).getNorm();
            time = getTime(dist);
        }
        // rep x+1
        Translation2d offset =
                new Translation2d(-vel.vxMetersPerSecond * time, -vel.vyMetersPerSecond * time);
        Translation2d vecToTarget = goal.plus(offset).minus(pos);
        dist = vecToTarget.getNorm();

        DataPoint data = get(dist, vecToTarget.getAngle().getDegrees());
        double timeError = data.time - time;
        Logger.recordOutput("Shooter/TimeError", timeError);

        return data;
    }
}
