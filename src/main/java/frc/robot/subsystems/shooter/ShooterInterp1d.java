package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.ConfigButtons;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class ShooterInterp1d {
    public static record DataPoint(
            double rpm, double angle, double hood, double time, double turretVel, double dist) {}

    // based on quick testing
    private static final double[] rpmTableReal = {
        2550, 2500, 2650, 2700, 3050, 3200, 3300, 3500, 3700, 4000
    };
    private static final double[] hoodAngleTableReal = {
        76, 70, 66.6, 55, 54.5, 50.5, 50.5, 50.5, 50.5, 50.5
    };
    private static final double[] timeTableReal = {
        0.58, 0.58, 0.68, 0.78, 0.91, 1.04, 1.30, 1.37, 1.44, 1.50
    };
    private static final double[] distAxisReal = {
        0.98, 1.317, 2.2, 2.75, 3.54, 3.91, 4.42, 5.03, 5.18, 5.79
    };
    // 139in 154in 180in 204in 17ft  19ft

    // for sim without drag
    private static final double[] rpmTableSim = {2860, 3403, 3963, 4473, 4938};
    private static final double[] hoodAngleTableSim = {73.3, 59.2, 54.1, 51.6, 50.2};
    private static final double[] timeTableSim = {0.29, 0.42, 0.64, 0.85, 1.0};
    private static final double[] distAxisSim = {0, 1.524, 3.048, 4.572, 6.096};

    // real passes with drag
    private static final double[] rpmTableRealPassing = {2000, 2800, 3300, 4500, 5300, 5500};
    private static final double[] hoodAngleTableRealPassing = {49.5, 49.5, 49.5, 49.5, 49.5, 49.5};
    private static final double[] timeTableRealPassing = {0.98, 1.3, 1.5, 1.57, 1.5, 1.38};
    private static final double[] distAxisPassingReal = {1, 4, 6, 9, 12, 15};
    // 49.5 for all
    // pairs
    /*3750rpm, 247in landed, 141in roll | soft dist, hard dist, "normal"
    5500 396     | 24.5' 33.5' 35.5'
    5000 372     | 25' 33'
    4500 360     | 23' 29
    4000         | 21' 24'
    3700 244 102 |
    3600 238 134 | 20.5' 21.5'
    3400 233 189 |
    3200 232 218 |
    3000 218 260 | 22' 24'
    2800 201 300 |
    2650 195 339 |
    2600 190 352 |
    2600 188 368 |
    2550 188 308 |
    2400 172 255 |
    2200 148 183 |


    */
    private static final double[] passLongData = {5000, 49.5, 1.5};
    private static final double[] passShortData = {2600, 49.5, 0.8};

    // for sim passing
    private static final double[] rpmTableSimPassing = {2600, 3400, 4200, 5000, 6000, 7100, 8200};
    private static final double[] hoodAngleTableSimPassing = {35, 37, 39, 40, 42, 44, 45};
    private static final double[] timeTableSimPassing = {
        0.214, 0.367, 0.527, 0.686, 0.854, 1.059, 1.19
    };
    private static final double[] distAxisPassingSim = {3, 6, 9, 12, 15, 18, 21};

    private static double[] rpmTable;
    private static double[] hoodAngleTable;
    private static double[] timeTable;
    private static double[] distAxis;

    private static double[] rpmTablePass;
    private static double[] hoodAngleTablePass;
    private static double[] timeTablePass;
    private static double[] distAxisPassing;

    // turret position lookahead
    private static final double turretPositionLookaheadDt = 0.02;
    // robot velocity lookahead
    private static final double dt = 0.02;

    NetworkTableEntry turretLookahead =
            NetworkTableInstance.getDefault().getTable("Tuning").getEntry("TurretLookahead");
    NetworkTableEntry robotLookahead =
            NetworkTableInstance.getDefault().getTable("Tuning").getEntry("RobotLookahead");

    public ShooterInterp1d() {
        if (Constants.currentMode == Mode.REAL || Constants.currentMode == Mode.REPLAY) {
            rpmTable = rpmTableReal;
            hoodAngleTable = hoodAngleTableReal;
            timeTable = timeTableReal;
            distAxis = distAxisReal;
            rpmTablePass = rpmTableRealPassing;
            hoodAngleTablePass = hoodAngleTableRealPassing;
            timeTablePass = timeTableRealPassing;
            distAxisPassing = distAxisPassingReal;
        } else {
            rpmTable = rpmTableSim;
            hoodAngleTable = hoodAngleTableSim;
            timeTable = timeTableSim;
            distAxis = distAxisSim;
            rpmTablePass = rpmTableSimPassing;
            hoodAngleTablePass = hoodAngleTableSimPassing;
            timeTablePass = timeTableSimPassing;
            distAxisPassing = distAxisPassingSim;
        }

        turretLookahead.setDouble(turretPositionLookaheadDt);
        robotLookahead.setDouble(dt);
    }

    public DataPoint get(
            double dist,
            double angle,
            double[] distAxis,
            double[] rpmTable,
            double[] hoodAngleTable,
            double[] timeTable,
            double turretVel) {
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

        return new DataPoint(rpm, angle, hood, time, turretVel, dist);
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

    // this solves move&shoot via multiple recursive lookups
    public DataPoint getReps(
            Translation2d goal,
            Pose2d pos,
            ChassisSpeeds vel,
            double distAxis[],
            double[] rpmTable,
            double[] hoodAngleTable,
            double[] timeTable) {

        // step1 project robot movement into the future
        Pose2d futureBotPose = pos.exp(vel.toTwist2d(robotLookahead.getDouble(dt)));
        Translation2d turretPos =
                futureBotPose
                        .getTranslation()
                        .plus(Constants.shooterLocOnBot.rotateBy(futureBotPose.getRotation()));

        // step2: offset goal location based on robot velocity
        //    a: turn robot rotation vel into turret vx vy and turret rotation
        Translation2d botVelXY = new Translation2d(vel.vxMetersPerSecond, vel.vyMetersPerSecond);
        // tangent velocity is in the direction of the turret with a magnitude of omega*R. Then
        // convert to field coords
        Translation2d turretXYfromRot =
                Constants.shooterLocOnBot
                        .rotateBy(
                                Rotation2d.fromRadians(
                                        Math.PI / 2.0
                                                + vel.omegaRadiansPerSecond
                                                        * robotLookahead.getDouble(dt)))
                        .rotateBy(futureBotPose.getRotation())
                        .times(vel.omegaRadiansPerSecond);
        Translation2d turretVelocity = botVelXY.plus(turretXYfromRot);
        Logger.recordOutput("Shooter/turretVelocity", turretVelocity);

        Translation2d velocityOffset;
        Translation2d vecToTarget;
        double dist;
        double tangentAngle;
        double tangentVelocity;
        double turretVel;
        DataPoint data;
        double turretAngle;
        double futureTurretAngle;

        // determine if pass or hub shot by comparison of timeTable reference
        if (false && (timeTable == timeTableRealPassing || timeTable == timeTableSimPassing)) {
            // select far or close pass based on control board switch
            // using the "field orient" switch (7)
            double[] passData; // contains rpm, angle, airtime
            if (ConfigButtons.driveStation.getHID().getRawButton(7)) {
                // on is long pass that rolls back
                passData = passLongData;
            } else {
                // off is short pass that rolls forward
                passData = passShortData;
            }

            // pass shot
            velocityOffset = turretVelocity.times(passData[2]);
            Logger.recordOutput("Shooter/velocityOffset", velocityOffset);

            vecToTarget = goal.plus(velocityOffset).minus(turretPos);
            dist = vecToTarget.getNorm();

            // calc the turret angular vel from the tangential turret velocity
            tangentAngle =
                    vecToTarget
                            .getAngle()
                            .plus(Rotation2d.kCCW_90deg)
                            .minus(
                                    Rotation2d.fromRadians(
                                            Math.atan2(
                                                    turretVelocity.getY(), turretVelocity.getX())))
                            .getRadians(); // tangent angle (b)
            tangentVelocity = Math.cos(tangentAngle) * turretVelocity.getNorm();
            turretVel = Math.toDegrees(tangentVelocity / dist);

            /*data =
                    get(
                            dist,
                            vecToTarget.getAngle().minus(futureBotPose.getRotation()).getDegrees(),
                            distAxis,
                            rpmTable,
                            hoodAngleTable,
                            timeTable,
                            turretVel);
            */
            turretAngle = vecToTarget.getAngle().minus(futureBotPose.getRotation()).getDegrees();
            // predict forward
            futureTurretAngle = turretAngle + turretVel * turretPositionLookaheadDt;
            data =
                    new DataPoint(
                            passData[0],
                            futureTurretAngle,
                            passData[1],
                            passData[2],
                            turretVel,
                            dist);

        } else {
            // hub shot
            //    b: offset goal based on turret velocity (initial time guess is the lowest shot
            // time)
            velocityOffset = turretVelocity.times(-timeTable[0]);
            Logger.recordOutput("Shooter/velocityOffset", velocityOffset);

            // rep 1
            dist = goal.plus(velocityOffset).minus(turretPos).getNorm();
            double time = getTime(dist, distAxis, timeTable);
            // rep 2 - x
            for (int i = 1; i < reps - 1; i++) {
                velocityOffset = turretVelocity.times(-time);

                dist = goal.plus(velocityOffset).minus(turretPos).getNorm();
                time = getTime(dist, distAxis, timeTable);
            }
            // rep x+1
            velocityOffset = turretVelocity.times(-time);
            vecToTarget = goal.plus(velocityOffset).minus(turretPos);
            dist = vecToTarget.getNorm();

            // calc the turret angular vel from the tangential turret velocity
            tangentAngle =
                    vecToTarget
                            .getAngle()
                            .plus(Rotation2d.kCCW_90deg)
                            .minus(
                                    Rotation2d.fromRadians(
                                            Math.atan2(
                                                    turretVelocity.getY(), turretVelocity.getX())))
                            .getRadians(); // tangent angle (b)
            tangentVelocity = Math.cos(tangentAngle) * turretVelocity.getNorm();
            turretVel = Math.toDegrees(tangentVelocity / dist);
            turretAngle = vecToTarget.getAngle().minus(futureBotPose.getRotation()).getDegrees();
            futureTurretAngle =
                    turretAngle + turretVel * turretLookahead.getDouble(turretPositionLookaheadDt);

            data =
                    get(
                            dist,
                            futureTurretAngle,
                            distAxis,
                            rpmTable,
                            hoodAngleTable,
                            timeTable,
                            turretVel);
            double timeError = data.time - time;
            Logger.recordOutput("Shooter/TimeError", timeError);
        }

        return data;
    }

    // do move&shoot via lookups of horizontal velocity
    public DataPoint getHV(
            Translation2d goal,
            Pose2d pos,
            ChassisSpeeds vel,
            double distAxis[],
            double[] rpmTable,
            double[] hoodAngleTable,
            double[] timeTable) {

        // step1: lookup the initial shot assuming a stationary robot
        //    a: project a future bot pose based on velocity
        Pose2d futureBotPose = pos.exp(vel.toTwist2d(dt));
        // TODO: should this rotate by future bot pose rotation or by bot pose rotation?
        Translation2d turretPos =
                futureBotPose
                        .getTranslation()
                        .plus(Constants.shooterLocOnBot.rotateBy(futureBotPose.getRotation()));

        //    b: lookup the stationary shot from that location
        double dist = goal.minus(turretPos).getNorm();
        DataPoint stationaryData = get(dist, 0, distAxis, rpmTable, hoodAngleTable, timeTable, 0);

        // step2: offset goal location based on robot velocity
        //    a: turn robot rotation vel into turret vx vy and turret rotation
        Translation2d botVelXY = new Translation2d(vel.vxMetersPerSecond, vel.vyMetersPerSecond);
        // tangent velocity is in the direction of the turret with a magnitude of omega*R. Then
        // convert to field coords
        Translation2d turretXYfromRot =
                Constants.shooterLocOnBot
                        .rotateBy(
                                Rotation2d.fromRadians(
                                        Math.PI / 2.0 + vel.omegaRadiansPerSecond * dt))
                        .rotateBy(futureBotPose.getRotation())
                        .times(vel.omegaRadiansPerSecond);
        Translation2d turretVelocity = botVelXY.plus(turretXYfromRot);
        Logger.recordOutput("Shooter/turretVelocity", turretVelocity);

        //    b: offset goal based on turret velocity
        Translation2d velocityOffset = turretVelocity.times(-stationaryData.time);
        Logger.recordOutput("Shooter/velocityOffset", velocityOffset);

        Translation2d vecToGoal = goal.minus(turretPos).plus(velocityOffset);
        Logger.recordOutput("Shooter/vecToGoal", vecToGoal);

        double movingDist = vecToGoal.getNorm();
        Logger.recordOutput("Shooter/movingDist", movingDist);

        // step3: calculate additional horizontal velocity required to make the adjusted shot in the
        // same amount of time
        double originalHorizVel = dist / stationaryData.time;
        double newHorizVel = movingDist / stationaryData.time;
        double rpmRatio =
                Math.cos(Math.toRadians(stationaryData.hood))
                        * stationaryData.rpm
                        / originalHorizVel;

        Logger.recordOutput("Shooter/newHorzVel", newHorizVel);
        // step4: assuming the same vertical velocity as the stationary shot, calculate a new exit
        // velocity and angle for the shot
        double newHorizRpm = newHorizVel * rpmRatio;
        double originalVertVel = Math.sin(Math.toRadians(stationaryData.hood)) * stationaryData.rpm;
        double newRpm = Math.sqrt(newHorizRpm * newHorizRpm + originalVertVel * originalVertVel);
        Logger.recordOutput("Shooter/newRpm", newRpm);
        double newHoodAngle = Math.toDegrees(Math.atan2(originalVertVel, newHorizRpm));
        Logger.recordOutput("Shooter/newHoodAngle", newHoodAngle);

        // turret angle, but robot relative
        double turretAngle = vecToGoal.getAngle().minus(futureBotPose.getRotation()).getDegrees();
        Logger.recordOutput("Shooter/vecToGoal", vecToGoal);

        // step5: return this data to the shoot command
        return new DataPoint(
                newRpm,
                turretAngle,
                newHoodAngle,
                stationaryData.time,
                Math.toDegrees(-vel.omegaRadiansPerSecond),
                0);
    }

    public DataPoint getHub(Translation2d goal, Pose2d pos, ChassisSpeeds vel) {
        return getReps(goal, pos, vel, distAxis, rpmTable, hoodAngleTable, timeTable);
        // return getHV(goal, pos, vel, distAxis, rpmTable, hoodAngleTable, timeTable);
    }

    public DataPoint getPass(Translation2d goal, Pose2d pos, ChassisSpeeds vel) {
        DataPoint data =
                getReps(
                        goal,
                        pos,
                        vel,
                        distAxisPassing,
                        rpmTablePass,
                        hoodAngleTablePass,
                        timeTablePass);

        return data;
    }
}
