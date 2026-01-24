package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;

public class ShooterInterp1d {
    public static record DataPoint(double rpm, double angle, double hood, double time) {}

    private static final double[] distAxis = {0, 1.524, 3.048, 4.572, 6.096};

    private static final double[] rpmTable = {2907, 3494, 4121, 4714, 5274};
    private static final double[] hoodAngleTable = {73.2, 58.7, 53.4, 50.8, 49.1};
    private static final double[] timeTable = {0.58, 0.78, 0.96, 1.12, 1.27};

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

    private static final int reps = 5;

    public DataPoint get(Translation2d goal, Translation2d pos, ChassisSpeeds vel) {
        // rep 1
        double dist = goal.minus(pos).getNorm();
        double time = getTime(dist);
        // rep 2 - x
        Translation2d offset;
        for (int i = 1; i < reps - 1; i++) {
            offset =
                    new Translation2d(-vel.vxMetersPerSecond * time, -vel.vyMetersPerSecond * time);
            dist = goal.plus(offset).minus(pos).getNorm();
            time = getTime(dist);
        }
        // rep x+1
        offset = new Translation2d(-vel.vxMetersPerSecond * time, -vel.vyMetersPerSecond * time);
        Translation2d vecToTarget = goal.plus(offset).minus(pos);
        dist = vecToTarget.getNorm();

        DataPoint data = get(dist, vecToTarget.getAngle().getDegrees());
        double timeError = data.time - time;
        Logger.recordOutput("Shooter/TimeError", timeError);

        return data;
    }
}
