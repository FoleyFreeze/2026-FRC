// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {

    private static final double DEADBAND = 0.1;
    private static final double ANGLE_KP = 8.0;
    private static final double ANGLE_KD = 0.4;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;

    private DriveCommands() {}

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(Translation2d.kZero, linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
                .getTranslation();
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    public static Command joystickDrive(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {
        return Commands.run(
                () -> {
                    // Get linear velocity
                    Translation2d linearVelocity =
                            getLinearVelocityFromJoysticks(
                                    xSupplier.getAsDouble(), ySupplier.getAsDouble());

                    // Apply rotation deadband
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                    // Square rotation value for more precise control
                    omega = Math.copySign(omega * omega, omega);

                    // Convert to field relative speeds & send command
                    ChassisSpeeds speeds =
                            new ChassisSpeeds(
                                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                    omega * drive.getMaxAngularSpeedRadPerSec());
                    boolean isFlipped =
                            DriverStation.getAlliance().isPresent()
                                    && DriverStation.getAlliance().get() == Alliance.Red;
                    drive.runVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    speeds,
                                    isFlipped
                                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                            : drive.getRotation()));
                },
                drive);
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     */
    public static Command driveAtAngleFFw(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Rotation2d> rotationSupplier,
            Supplier<Double> rotationFFw) {

        // Create PID controller
        ProfiledPIDController angleController =
                new ProfiledPIDController(
                        ANGLE_KP,
                        0.0,
                        ANGLE_KD,
                        new TrapezoidProfile.Constraints(
                                ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return Commands.run(
                        () -> {
                            // Get linear velocity
                            Translation2d linearVelocity =
                                    getLinearVelocityFromJoysticks(
                                            xSupplier.getAsDouble(), ySupplier.getAsDouble());

                            // Calculate angular speed
                            Rotation2d targetRotation = rotationSupplier.get();
                            Logger.recordOutput("Drive/TargetRotation", targetRotation);
                            double omega =
                                    angleController.calculate(
                                            drive.getRotation().getRadians(),
                                            new TrapezoidProfile.State(
                                                    targetRotation.getRadians(),
                                                    rotationFFw.get())); // in radians
                            // (radians/second)

                            // Convert to field relative speeds & send command
                            ChassisSpeeds speeds =
                                    new ChassisSpeeds(
                                            linearVelocity.getX()
                                                    * drive.getMaxLinearSpeedMetersPerSec(),
                                            linearVelocity.getY()
                                                    * drive.getMaxLinearSpeedMetersPerSec(),
                                            omega);
                            boolean isFlipped =
                                    DriverStation.getAlliance().isPresent()
                                            && DriverStation.getAlliance().get() == Alliance.Red;
                            drive.runVelocityTurretCenter(
                                    ChassisSpeeds.fromFieldRelativeSpeeds(
                                            speeds,
                                            isFlipped
                                                    ? drive.getRotation()
                                                            .plus(new Rotation2d(Math.PI))
                                                    : drive.getRotation()));
                        },
                        drive)

                // Reset PID controller when command starts
                .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
    }

    public static Command joystickDriveAtAngle(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Rotation2d> rotationSupplier) {

        // Create PID controller
        ProfiledPIDController angleController =
                new ProfiledPIDController(
                        ANGLE_KP,
                        0.0,
                        ANGLE_KD,
                        new TrapezoidProfile.Constraints(
                                ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return Commands.run(
                        () -> {
                            // Get linear velocity
                            Translation2d linearVelocity =
                                    getLinearVelocityFromJoysticks(
                                            xSupplier.getAsDouble(), ySupplier.getAsDouble());

                            // Calculate angular speed
                            Rotation2d targetRotation = rotationSupplier.get();
                            Logger.recordOutput("Drive/TargetRotation", targetRotation);
                            double omega =
                                    angleController.calculate(
                                            drive.getRotation().getRadians(),
                                            targetRotation.getRadians());

                            // Convert to field relative speeds & send command
                            ChassisSpeeds speeds =
                                    new ChassisSpeeds(
                                            linearVelocity.getX()
                                                    * drive.getMaxLinearSpeedMetersPerSec(),
                                            linearVelocity.getY()
                                                    * drive.getMaxLinearSpeedMetersPerSec(),
                                            omega);
                            boolean isFlipped =
                                    DriverStation.getAlliance().isPresent()
                                            && DriverStation.getAlliance().get() == Alliance.Red;
                            drive.runVelocity(
                                    ChassisSpeeds.fromFieldRelativeSpeeds(
                                            speeds,
                                            isFlipped
                                                    ? drive.getRotation()
                                                            .plus(new Rotation2d(Math.PI))
                                                    : drive.getRotation()));
                        },
                        drive)

                // Reset PID controller when command starts
                .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
    }

    public static Command driveToPoint(RobotContainer r, Supplier<Pose2d> supplier) {
        PIDController pidX = new PIDController(8, 0, 0);
        PIDController pidY = new PIDController(8, 0, 0);
        final double POS_TOL = Units.inchesToMeters(0.5);
        final double POS_MAX_VEL = 1; // may change
        final double POS_MAX_TIME = 2;
        double[] error = new double[1];
        Timer timer = new Timer();

        return Commands.run(
                        () -> {
                            // gets where go
                            Pose2d target = supplier.get();
                            Pose2d meas = r.drive.getPose(); // may do local pose later
                            // error calculation
                            Translation2d pointErr =
                                    target.getTranslation().minus(meas.getTranslation());
                            error[0] = pointErr.getNorm();
                            Logger.recordOutput("Odometry/PointErr", pointErr);
                            Logger.recordOutput("Odometry/PointErrNorm", error[0]);
                            // using error to calc the vel
                            double xVel = pidX.calculate(meas.getX(), target.getX());
                            double yVel = pidY.calculate(meas.getY(), target.getY());
                            // limits vel (its turtle time)
                            xVel = MathUtil.clamp(xVel, -POS_MAX_VEL, POS_MAX_VEL);
                            yVel = MathUtil.clamp(yVel, -POS_MAX_VEL, POS_MAX_VEL);

                            // TODO: run angle PID in parallel
                            r.drive.runVelocity(
                                    ChassisSpeeds.fromFieldRelativeSpeeds(
                                            new ChassisSpeeds(xVel, yVel, 0),
                                            r.drive.getRotation()));
                        },
                        r.drive)
                .until(() -> error[0] < POS_TOL || timer.hasElapsed(POS_MAX_TIME))
                .beforeStarting(
                        () -> {
                            pidX.reset();
                            pidY.reset();
                            timer.restart();
                        })
                .andThen(
                        new InstantCommand(
                                () -> r.drive.runVelocity(new ChassisSpeeds()), r.drive));
    }

    public static Command driveToPointNoSupply(RobotContainer r, Pose2d target) {
        PIDController pidX = new PIDController(8, 0, 0);
        PIDController pidY = new PIDController(8, 0, 0);
        final double POS_TOL = Units.inchesToMeters(0.5);
        final double POS_MAX_VEL = 1; // may change
        final double POS_MAX_TIME = 2;
        double[] error = new double[1];
        Timer timer = new Timer();

        return Commands.run(
                        () -> {
                            // gets where go;
                            Pose2d meas = r.drive.getPose(); // may do local pose later
                            // error calculation
                            Translation2d pointErr =
                                    target.getTranslation().minus(meas.getTranslation());
                            error[0] = pointErr.getNorm();
                            Logger.recordOutput("Odometry/PointErr", pointErr);
                            Logger.recordOutput("Odometry/PointErrNorm", error[0]);
                            // using error to calc the vel
                            double xVel = pidX.calculate(meas.getX(), target.getX());
                            double yVel = pidY.calculate(meas.getY(), target.getY());
                            // limits vel (its turtle time)
                            xVel = MathUtil.clamp(xVel, -POS_MAX_VEL, POS_MAX_VEL);
                            yVel = MathUtil.clamp(yVel, -POS_MAX_VEL, POS_MAX_VEL);

                            // TODO: run angle PID in parallel
                            r.drive.runVelocity(
                                    ChassisSpeeds.fromFieldRelativeSpeeds(
                                            new ChassisSpeeds(xVel, yVel, 0),
                                            r.drive.getRotation()));
                        },
                        r.drive)
                .until(() -> error[0] < POS_TOL || timer.hasElapsed(POS_MAX_TIME))
                .beforeStarting(
                        () -> {
                            pidX.reset();
                            pidY.reset();
                            timer.restart();
                        })
                .andThen(
                        new InstantCommand(
                                () -> r.drive.runVelocity(new ChassisSpeeds()), r.drive));
    }

    private static ChassisSpeeds prevSpeeds = new ChassisSpeeds();
    private static double ACCEL_LIMIT = 2.0 * 0.02; // limit to 2m/s^2
    private static double TURN_LIMIT = Units.degreesToRadians(90); // limit to 90deg/sec

    public static Command slowDrive(CommandXboxController controller, Drive drive) {
        return Commands.run(
                () -> {
                    // Get linear velocity
                    Translation2d linearVelocity =
                            getLinearVelocityFromJoysticks(
                                    -controller.getLeftY(), -controller.getLeftX());

                    // Apply rotation deadband
                    double omega = MathUtil.applyDeadband(-controller.getRightX(), DEADBAND);

                    // Convert to field relative speeds & send command
                    ChassisSpeeds speeds =
                            new ChassisSpeeds(
                                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                    omega * drive.getMaxAngularSpeedRadPerSec());

                    speeds.vxMetersPerSecond =
                            gradLim(
                                    speeds.vxMetersPerSecond,
                                    prevSpeeds.vxMetersPerSecond,
                                    ACCEL_LIMIT);
                    speeds.vyMetersPerSecond =
                            gradLim(
                                    speeds.vyMetersPerSecond,
                                    prevSpeeds.vyMetersPerSecond,
                                    ACCEL_LIMIT);
                    speeds.omegaRadiansPerSecond =
                            MathUtil.clamp(speeds.omegaRadiansPerSecond, -TURN_LIMIT, TURN_LIMIT);

                    boolean isFlipped =
                            DriverStation.getAlliance().isPresent()
                                    && DriverStation.getAlliance().get() == Alliance.Red;
                    drive.runVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    speeds,
                                    isFlipped
                                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                            : drive.getRotation()));

                    prevSpeeds = speeds;
                },
                drive);
    }

    public static double gradLim(double value, double prevValue, double limit) {
        if (value - prevValue > limit) {
            return value += limit;
        } else if (value - prevValue < -limit) {
            return value -= limit;
        } else {
            return value;
        }
    }
}
