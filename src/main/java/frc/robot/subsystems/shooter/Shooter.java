package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.ShooterCommands.Thing;
import frc.robot.subsystems.shooter.ShooterInterp1d.DataPoint;
import frc.robot.util.Util;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    RobotContainer r;
    public boolean isTurret = false; // TODO: cal for turret

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final ShooterInterp1d lerp = new ShooterInterp1d();

    private double rpmTarget, hoodTarget, turretTarget, botAngleTarget;

    public static final double minHoodAngle = 48;
    public static final double maxHoodAngle = 87;

    public static enum ShootMode {
        HUB,
        PASS,
        MANUAL,
    }

    public ShootMode shootMode = ShootMode.MANUAL;

    public static enum ManualShotLoc {
        CLIMB, // x
        FRONT_HUB, // y
    }

    double climbManualSpeed = 1;

    double frontHubManualSpeed = 1;

    double cornerManualSpeed = 1;

    private static ManualShotLoc manualShotLocState = ManualShotLoc.FRONT_HUB;

    public Translation2d goal = new Translation2d();

    public static enum MissReason {
        NONE,
        TURRET_ANGLE,
        ROBOT_ANGLE,
        HOOD_ANGLE,
        WHEEL_SPEED,
        HUB_INTERSECTION,
    }

    @AutoLogOutput(key = "Shooter/MissReasonShooter")
    public MissReason missReason = MissReason.NONE;

    public Shooter(ShooterIO io, RobotContainer r) {
        this.io = io;
        this.r = r;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public double getTurretAngle() {
        return inputs.turretPosition;
    }

    public double getHoodAngle() {
        return inputs.hoodPosition;
    }

    public Command prime() {
        return new InstantCommand(
                () -> {
                    io.wheelPower(1);
                },
                this);
    }

    public Command stop() {
        return new RunCommand(
                () -> {
                    io.wheelPower(0);
                    io.setHoodAngle(maxHoodAngle);
                },
                this);
    }

    // deprecated
    public Command cameraShoot(Supplier<Pose2d> botPose, Supplier<ChassisSpeeds> botVel) {
        return new RunCommand(() -> goalPrime(botPose.get(), botVel.get()), this);
    }

    private Translation2d getClosestGoal(Pose2d botLoc) {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            if (botLoc.getX() < FieldConstants.HorizontalLines.starting) {
                // shoot@goAL
                goal = FieldConstants.Hub.center;
                shootMode = ShootMode.HUB;
            } else {
                // pass
                shootMode = ShootMode.PASS;
                if (botLoc.getY() < FieldConstants.fieldWidth / 2) {
                    goal = FieldConstants.Locations.passRight;
                } else {
                    goal = FieldConstants.Locations.passLeft;
                }
            }
        } else {
            if (botLoc.getX()
                    > FieldConstants.fieldLength - FieldConstants.HorizontalLines.starting) {
                // shoot@goAL
                goal = FieldConstants.flip(FieldConstants.Hub.center);
                shootMode = ShootMode.HUB;
            } else {
                // pass
                shootMode = ShootMode.PASS;
                if (botLoc.getY() < FieldConstants.fieldWidth / 2) {
                    goal = FieldConstants.flip(FieldConstants.Locations.passRight);
                }
                goal = FieldConstants.flip(FieldConstants.Locations.passLeft);
            }
        }
        return goal;
    }

    double lastPredFlightTime = 0;

    public double getLastFlightTime() {
        return lastPredFlightTime;
    }

    public void goalPrime(Pose2d botLoc, ChassisSpeeds botVel) {

        // 0 what are we shooting at? (goal vs pass)
        Translation2d goal = getClosestGoal(botLoc);

        // 1 call the lerp
        DataPoint setpoints;
        if (shootMode == ShootMode.HUB) {
            setpoints = lerp.getHub(goal, botLoc, botVel);
        } else {
            setpoints = lerp.getPass(goal, botLoc, botVel);
        }
        lastPredFlightTime = setpoints.time();

        // 2 use setpoints from lerp to set motors
        double angleSetpoint = setpoints.angle();
        Logger.recordOutput("Shooter/RawTurretSetpoint", angleSetpoint);
        Logger.recordOutput("Shooter/TurretVelocity", setpoints.turretVel());
        Logger.recordOutput("Shooter/HoodSetpoint", setpoints.hood());
        Logger.recordOutput("Shooter/RPMSetpoint", setpoints.rpm());
        Logger.recordOutput("Shooter/Distance", setpoints.dist());

        hoodTarget = setpoints.hood();
        rpmTarget = setpoints.rpm();
        manageTurretWrap(angleSetpoint, setpoints.turretVel());
        io.setHoodAngle(setpoints.hood());
        io.setSpeed(setpoints.rpm());
    }

    // for the current no-turret
    public void newPrime(
            Translation2d localgoal,
            Pose2d botLoc,
            Thing<Rotation2d> rotationThing,
            Thing<Double> velocityThing) {
        // 0 what are we shooting at? (goal vs pass)
        ChassisSpeeds botVel;

        botVel = r.drive.getFieldVelocity();
        this.goal = FieldConstants.flipIfRed(localgoal);

        // 1 call the lerp
        DataPoint setpoints;
        if (localgoal == FieldConstants.Hub.center) {
            setpoints = lerp.getHub(this.goal, botLoc, botVel);
            shootMode = ShootMode.HUB;
        } else {
            setpoints = lerp.getPass(this.goal, botLoc, botVel);
            shootMode = ShootMode.PASS;
        }
        lastPredFlightTime = setpoints.time();

        // 2 use setpoints from lerp to set motors
        double angleSetpoint = setpoints.angle() - 90 + r.drive.getRotation().getDegrees();
        botAngleTarget = angleSetpoint;
        Logger.recordOutput("Shooter/RawTurretSetpoint", angleSetpoint);
        Logger.recordOutput("Shooter/TurretVelocity", setpoints.turretVel());
        Logger.recordOutput("Shooter/HoodSetpoint", setpoints.hood());
        Logger.recordOutput("Shooter/RPMSetpoint", setpoints.rpm());
        Logger.recordOutput("Shooter/Distance", setpoints.dist());

        hoodTarget = setpoints.hood();
        rpmTarget = setpoints.rpm();
        // manageTurretWrap(angleSetpoint, setpoints.turretVel());
        rotationThing.accept(Rotation2d.fromDegrees(angleSetpoint));
        velocityThing.accept(Math.toRadians(setpoints.turretVel()));
        io.setHoodAngle(setpoints.hood());
        io.setSpeed(setpoints.rpm());
    }

    // for the eventual turret
    public void newPrime(Translation2d goal, Pose2d botLoc) {
        // 0 what are we shooting at? (goal vs pass)
        ChassisSpeeds botVel;

        botVel = r.drive.getChassisSpeeds();
        this.goal = FieldConstants.flipIfRed(goal);

        // 1 call the lerp
        DataPoint setpoints;
        if (goal == FieldConstants.Hub.center) {
            setpoints = lerp.getHub(this.goal, botLoc, botVel);
            shootMode = ShootMode.HUB;
        } else {
            setpoints = lerp.getPass(this.goal, botLoc, botVel);
            shootMode = ShootMode.PASS;
        }
        lastPredFlightTime = setpoints.time();

        // 2 use setpoints from lerp to set motors
        double angleSetpoint = setpoints.angle();
        Logger.recordOutput("Shooter/RawTurretSetpoint", angleSetpoint);
        Logger.recordOutput("Shooter/TurretVelocity", setpoints.turretVel());
        Logger.recordOutput("Shooter/HoodSetpoint", setpoints.hood());
        Logger.recordOutput("Shooter/RPMSetpoint", setpoints.rpm());
        Logger.recordOutput("Shooter/Distance", setpoints.dist());

        hoodTarget = setpoints.hood();
        rpmTarget = setpoints.rpm();
        manageTurretWrap(angleSetpoint, setpoints.turretVel());
        io.setHoodAngle(setpoints.hood());
        io.setSpeed(setpoints.rpm());
    }

    public void manageTurretWrap(double angle, double velocity) {
        double trueAngle = angle - Constants.turretAngleOffset;
        double normAngle = Util.floorMod(trueAngle, 360);
        double delta = normAngle - (inputs.turretPosition % 360);

        double shortDelta;
        if (delta > 180) {
            shortDelta = delta - 360;
        } else if (delta < -180) {
            shortDelta = delta + 360;
        } else {
            shortDelta = delta;
        }

        double setPoint = shortDelta + (inputs.turretPosition);
        if (setPoint > Constants.maximumTurretAngle) {
            setPoint -= 360;
        } else if (setPoint < 0) {
            setPoint += 360;
        }

        Logger.recordOutput("Shooter/TurretSetpoint", setPoint);
        turretTarget = setPoint;
        io.setTurretAngle(setPoint, velocity);
    }

    public void setManualGoal(ManualShotLoc loc) {
        manualShotLocState = loc;
    }

    public Command manualPrimeCmd() {
        return new RunCommand(() -> newPrime(FieldConstants.Hub.center, getManualPose()), this);
    }

    public Pose2d getManualPose() {
        Pose2d fakeLoc;

        switch (manualShotLocState) {
            case CLIMB:
                fakeLoc = FieldConstants.Locations.locationClimbShoot;
                break;
            case FRONT_HUB:
                fakeLoc = FieldConstants.Locations.locationHubShoot;
                break;
            default:
                fakeLoc = FieldConstants.Locations.locationClimbShoot;
        }
        return fakeLoc;
    }

    public boolean willHitHub(Pose2d botLoc) {
        // ignore if not passing
        if (shootMode != ShootMode.PASS) return false;

        // order our x values
        double startX, endX, startY, endY;
        if (botLoc.getX() > goal.getX()) {
            startX = goal.getX();
            endX = botLoc.getX();
            startY = goal.getY();
            endY = botLoc.getY();
        } else {
            endX = goal.getX();
            startX = botLoc.getX();
            endY = goal.getY();
            startY = botLoc.getY();
        }

        // do the lerp
        double xInterp =
                MathUtil.inverseInterpolate(
                        startX, endX, FieldConstants.flipIfRed(FieldConstants.Hub.backLeft).getX());
        double hubY = MathUtil.interpolate(startY, endY, xInterp);

        // determine if there is an intersection
        return hubY < FieldConstants.Hub.backLeft.getY()
                && hubY > FieldConstants.Hub.backRight.getY();
    }

    public boolean wontMiss(Pose2d botLoc) {
        // allow wider thresholds for passing
        double speedThresh = shootMode == ShootMode.HUB ? 150 : 300;
        double angleThresh = shootMode == ShootMode.HUB ? 5 : 10;

        if (!isWithin(rpmTarget, inputs.wheelVelocity, speedThresh)) {
            missReason = MissReason.WHEEL_SPEED;
            return false;
        } else if (isTurret && !isWithin(turretTarget, inputs.turretPosition, angleThresh)) {
            missReason = MissReason.TURRET_ANGLE;
            return false;
        } else if (!isTurret
                && !isWithin(
                        r.drive
                                .getRotation()
                                .minus(Rotation2d.fromDegrees(botAngleTarget))
                                .getDegrees(),
                        0,
                        angleThresh)) {
            missReason = MissReason.ROBOT_ANGLE;
            return false;
        } else if (!isWithin(hoodTarget, inputs.hoodPosition, angleThresh)) {
            missReason = MissReason.HOOD_ANGLE;
            return false;
        } else if (willHitHub(botLoc)) {
            missReason = MissReason.HUB_INTERSECTION;
            return false;
        } else {
            missReason = MissReason.NONE;
            return true;
        }
    }

    public boolean isWithin(double target, double actual, double range) {
        double delta = actual - target;
        return Math.abs(delta) < range;
    }

    public Command manualShoot(DoubleSupplier turretPower, DoubleSupplier hoodPower) {
        return new RunCommand(
                () -> {
                    io.hoodPower(hoodPower.getAsDouble());
                    io.turretPower(turretPower.getAsDouble());
                },
                this);
    }

    public double getAngleCRT(double e1, double e2) {
        final double t_teeth = 400;
        final double e1_teeth = 25;
        final double e2_teeth = 27;
        final double maxTurretDegrees = 400;
        // maximum number of e1 & e2 rotations worth considering (dependent on how many degrees of
        // turret angle you can measure)
        final int arraySize =
                (int)
                        Math.ceil(
                                (maxTurretDegrees / 360.0)
                                        * t_teeth
                                        / (Math.min(e1_teeth, e2_teeth)));

        // stores all possible turret angles that match measured encoder angle
        double[] e1PossibleAngles = new double[arraySize]; // in degrees
        double[] e2PossibleAngles = new double[arraySize];

        double minError =
                1e6; // set to a rly big number so it will not be less than the new min error
        // (ln260)
        double turretAngle = 0;

        // fills arrays e1 & e2 with possible turretAngles
        for (int i = 0; i < arraySize; i++) {
            e1PossibleAngles[i] = (i + (e1 / 360)) * (e1_teeth / t_teeth) * 360;
            e2PossibleAngles[i] = (i + (e2 / 360)) * (e2_teeth / t_teeth) * 360;
        }
        // searching for the angle that both encoders agree on
        for (int i_e1 = 0; i_e1 < arraySize; i_e1++) {
            for (int i_e2 = 0; i_e2 < arraySize; i_e2++) {
                double error = Math.abs(e1PossibleAngles[i_e1] - e2PossibleAngles[i_e2]);
                if (error < minError) {
                    minError = error;
                    turretAngle = (e1PossibleAngles[i_e1] + e2PossibleAngles[i_e2]) / 2.0;
                }
            }
        }
        Logger.recordOutput("Shooter/CRT_TurretAngle", turretAngle);
        Logger.recordOutput("Shooter/CRT_MinError", minError);

        return turretAngle;
    }
}
