package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class ShooterInterp1d {
    public static record DataPoint(double rpm, double angle, double hood, double time) {}

    private static final double[] distAxis = {0, 1.524, 3.048, 4.572, 6.096};

    private static final double[] distAxisPassing = {3, 6, 9, 12, 15, 18, 21};

    // with modeled drag
    private static final double[] rpmTableReal = {2907, 3494, 4121, 4714, 5274};
    private static final double[] hoodAngleTableReal = {73.2, 58.7, 53.4, 50.8, 49.1};
    private static final double[] timeTableReal = {0.58, 0.78, 0.96, 1.12, 1.27};

    // for sim without drag
    private static final double[] rpmTableSim = {2860, 3403, 3963, 4473, 4938};
    private static final double[] hoodAngleTableSim = {73.3, 59.2, 54.1, 51.6, 50.2};
    private static final double[] timeTableSim = {0.59, 0.78, 0.96, 1.12, 1.26};

    // real passes with drag
    private static final double[] rpmTableRealPassing = {1000, 1600, 2200, 2800, 3400, 4000, 4600};
    private static final double[] hoodAngleTableRealPassing = {35, 37, 39, 40, 42, 44, 45};
    private static final double[] timeTableRealPassing = {
        0.214, 0.367, 0.527, 0.686, 0.854, 1.059, 1.19
    };

    // for sim passing
    private static final double[] rpmTableSimPassing = {2600, 3400, 4200, 5000, 6000, 7100, 8200};
    private static final double[] hoodAngleTableSimPassing = {35, 37, 39, 40, 42, 44, 45};
    private static final double[] timeTableSimPassing = {
        0.214, 0.367, 0.527, 0.686, 0.854, 1.059, 1.19
    };

    private static double[] rpmTable;
    private static double[] hoodAngleTable;
    private static double[] timeTable;

    private static double[] rpmTablePass;
    private static double[] hoodAngleTablePass;
    private static double[] timeTablePass;

    public ShooterInterp1d() {
        if (Robot.isReal()) {
            rpmTable = rpmTableReal;
            hoodAngleTable = hoodAngleTableReal;
            timeTable = timeTableReal;
            rpmTablePass = rpmTableRealPassing;
            hoodAngleTablePass = hoodAngleTableRealPassing;
            timeTablePass = timeTableRealPassing;
        } else {
            rpmTable = rpmTableSim;
            hoodAngleTable = hoodAngleTableSim;
            timeTable = timeTableSim;
            rpmTablePass = rpmTableSimPassing;
            hoodAngleTablePass = hoodAngleTableSimPassing;
            timeTablePass = timeTableSimPassing;
        }
    }

    public DataPoint get(
            double dist,
            double angle,
            double[] distAxis,
            double[] rpmTable,
            double[] hoodAngleTable,
            double[] timeTable) {
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

    public double getTime(double dist, double[] distAxis, double[] timeTable) {
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

    //this solves move&shoot via multiple recursive lookups
    public DataPoint getReps(
            Translation2d goal,
            Pose2d pos,
            ChassisSpeeds vel,
            double distAxis[],
            double[] rpmTable,
            double[] hoodAngleTable,
            double[] timeTable) {
        
        Translation2d bot = pos.getTranslation();
        // rep 1
        double dist = goal.minus(bot).getNorm();
        double time = getTime(dist, distAxis, timeTable);
        // rep 2 - x
        Translation2d offset;
        for (int i = 1; i < reps - 1; i++) {
            offset =
                    new Translation2d(-vel.vxMetersPerSecond * time, -vel.vyMetersPerSecond * time);
            dist = goal.plus(offset).minus(bot).getNorm();
            time = getTime(dist, distAxis, timeTable);
        }
        // rep x+1
        offset = new Translation2d(-vel.vxMetersPerSecond * time, -vel.vyMetersPerSecond * time);
        Translation2d vecToTarget = goal.plus(offset).minus(bot);
        dist = vecToTarget.getNorm();

        DataPoint data =
                get(
                        dist,
                        vecToTarget.getAngle().getDegrees(),
                        distAxis,
                        rpmTable,
                        hoodAngleTable,
                        timeTable);
        double timeError = data.time - time;
        Logger.recordOutput("Shooter/TimeError", timeError);

        return data;
    }

    //how much velocity lookahead to do
    static final double dt = 0.02;

    //do move&shoot via lookups of horizontal velocity
    public DataPoint getHV(Translation2d goal,
            Pose2d pos,
            ChassisSpeeds vel,
            double distAxis[],
            double[] rpmTable,
            double[] hoodAngleTable,
            double[] timeTable) {
        
        //step1: lookup the initial shot assuming a stationary robot
        //    a: project a future bot pose based on velocity
        Pose2d futureBotPose = pos.exp(vel.toTwist2d(dt));
        //TODO: should this rotate by future bot pose rotation or by bot pose rotation?
        Translation2d turretPos = futureBotPose.getTranslation().plus(Constants.shooterLocOnBot.rotateBy(futureBotPose.getRotation()));

        //    b: lookup the stationary shot from that location
        double dist = goal.minus(turretPos).getNorm();
        DataPoint stationaryData =
                get(dist,
                        0,
                        distAxis,
                        rpmTable,
                        hoodAngleTable,
                        timeTable);

        //step2: offset goal location based on robot velocity
        //    a: turn robot rotation vel into turret vx vy and turret rotation
        Translation2d botVelXY = new Translation2d(vel.vxMetersPerSecond, vel.vyMetersPerSecond);
        //tangent velocity is in the direction of the turret with a magnitude of omega*R. Then convert to field coords
        Translation2d turretXYfromRot = Constants.shooterLocOnBot.rotateBy(Rotation2d.fromRadians(Math.PI/2.0 + vel.omegaRadiansPerSecond*dt)).rotateBy(futureBotPose.getRotation());
        Translation2d turretVelocity = botVelXY.plus(turretXYfromRot);

        //    b: offset goal based on turret velocity
        Translation2d velocityOffset = turretVelocity.times(-stationaryData.time);
        Translation2d vecToGoal = goal.minus(futureBotPose.getTranslation()).plus(velocityOffset);
        double movingDist = vecToGoal.getNorm();

        //step3: calculate additional horizontal velocity required to make the adjusted shot in the same amount of time
        //double originalHorizVel = dist / stationaryData.time;
        double newHorizVel = movingDist / stationaryData.time;

        //step4: assuming the same vertical velocity as the stationary shot, calculate a new exit velocity and angle for the shot
        double originalVertVel = Math.sin(Math.toRadians(stationaryData.hood)) * stationaryData.rpm;
        double newRpm = Math.sqrt(newHorizVel*newHorizVel + originalVertVel*originalVertVel);
        double newHoodAngle = Math.toDegrees(Math.atan2(originalVertVel, newHorizVel));
        //turret angle, but robot relative
        double turretAngle = vecToGoal.getAngle().minus(futureBotPose.getRotation()).getDegrees();

        //step5: return this data to the shoot command
        return new DataPoint(newRpm, turretAngle, newHoodAngle, stationaryData.time);
    }

    public DataPoint getHub(Translation2d goal, Pose2d pos, ChassisSpeeds vel) {
        //return getReps(goal, pos, vel, distAxis, rpmTable, hoodAngleTable, timeTable);
        return getHV(goal, pos, vel, distAxis, rpmTable, hoodAngleTable, timeTable);
    }

    public DataPoint getPass(Translation2d goal, Pose2d pos, ChassisSpeeds vel) {
        // return getReps(
        //         goal, pos, vel, distAxisPassing, rpmTablePass, hoodAngleTablePass, timeTablePass);
        return getHV(
                goal, pos, vel, distAxisPassing, rpmTablePass, hoodAngleTablePass, timeTablePass);
    }
}
