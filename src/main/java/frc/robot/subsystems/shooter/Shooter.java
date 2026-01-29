package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.ShooterInterp1d.DataPoint;
import frc.robot.util.Util;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final ShooterInterp1d lerp = new ShooterInterp1d();

    private double rpmTarget, hoodTarget, turretTarget;

    public static enum ShootMode {
        HUB,
        PASS,
        MANUAL,
    }

    public ShootMode shootMode = ShootMode.MANUAL;
    public Translation2d goal = new Translation2d();

    public static enum MissReason {
        NONE,
        TURRET_ANGLE,
        HOOD_ANGLE,
        WHEEL_SPEED,
        HUB_INTERSECTION,
    }

    @AutoLogOutput(key = "Shooter/MissReasonShooter")
    public MissReason missReason = MissReason.NONE;

    public Shooter(ShooterIO io) {
        this.io = io;
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
        return new RunCommand(
                () -> {
                    io.wheelPower(1);
                    shootMode = ShootMode.MANUAL;
                },
                this);
    }

    public Command stop() {
        return new RunCommand(() -> io.wheelPower(0), this);
    }

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
                    goal = FieldConstants.passRight;
                } else {
                    goal = FieldConstants.passLeft;
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
                    goal = FieldConstants.flip(FieldConstants.passRight);
                }
                goal = FieldConstants.flip(FieldConstants.passLeft);
            }
        }
        return goal;
    }

    public void goalPrime(Pose2d botLoc, ChassisSpeeds botVel) {

        // 0 what are we shooting at? (goal vs pass)
        Translation2d goal = getClosestGoal(botLoc);

        // 1 collect the data to call the lerp

        Translation2d bot =
                botLoc.getTranslation()
                        .plus(Constants.shooterLocOnBot.rotateBy(botLoc.getRotation()));

        // 2 call the lerp
        DataPoint setpoints;
        if (shootMode == ShootMode.HUB) {
            setpoints = lerp.getHub(goal, bot, botVel);
        } else {
            setpoints = lerp.getPass(goal, bot, botVel);
        }

        // 3 use setpoints from lerp to set motors
        double angleSetpoint = setpoints.angle() - botLoc.getRotation().getDegrees();
        Logger.recordOutput("Shooter/RawTurretSetpoint", angleSetpoint);
        Logger.recordOutput("Shooter/HoodSetpoint", setpoints.hood());
        Logger.recordOutput("Shooter/RPMSetpoint", setpoints.rpm());

        hoodTarget = setpoints.hood();
        rpmTarget = setpoints.rpm();
        manageTurretWrap(angleSetpoint);
        io.setHoodAngle(setpoints.hood());
        io.setSpeed(setpoints.rpm());
    }

    public void manageTurretWrap(double angle) {
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
        io.setTurretAngle(setPoint);
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
        //allow wider thresholds for passing
        double speedThresh = shootMode == ShootMode.HUB ? 50 : 100;
        double angleThresh = shootMode == ShootMode.HUB ? 0.5 : 1;

        if (!isWithin(rpmTarget, inputs.wheelVelocity, speedThresh)) {
            missReason = MissReason.WHEEL_SPEED;
            return false;
        } else if (!isWithin(turretTarget, inputs.turretPosition, angleThresh)) {
            missReason = MissReason.TURRET_ANGLE;
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
}
