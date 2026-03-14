package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PathFinderCommand;
import frc.robot.commands.ShooterCommands;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class ChoreoAutos {
    RobotContainer r;
    AutoFactory autoFactory;

    public ChoreoAutos(RobotContainer r) {
        this.r = r;
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        autoFactory =
                new AutoFactory(
                        r.drive::getPose, // A function that returns the current robot pose
                        (pose) -> {}, // dont reset the pose for now
                        // r.drive::setPose, // A function that resets the current robot pose to the
                        // provided Pose2d
                        this::followTrajectory, // The drive subsystem trajectory follower
                        true, // If alliance flipping should be enabled
                        r.drive // The drive subsystem
                        );
    }

    public Command loadTraj(String name) {
        return autoFactory.trajectoryCmd(name);
    }

    // unique choreo PIDs (path follower PIDs are in main drive subsystem file)
    private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);

    public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = r.drive.getPose();

        // log the error data stuff
        Logger.recordOutput("Choreo/Target", sample.getPose());
        Logger.recordOutput("Choreo/Xerr", pose.getX() - sample.x);
        Logger.recordOutput("Choreo/Yerr", pose.getY() - sample.y);
        Logger.recordOutput(
                "Choreo/Rerr", pose.getRotation().minus(Rotation2d.fromRadians(sample.heading)));

        // Generate the next speeds for the robot
        ChassisSpeeds speeds =
                new ChassisSpeeds(
                        sample.vx + xController.calculate(pose.getX(), sample.x),
                        sample.vy + yController.calculate(pose.getY(), sample.y),
                        sample.omega
                                + headingController.calculate(
                                        pose.getRotation().getRadians(), sample.heading));

        // Apply the generated speeds
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, r.drive.getRotation());
        r.drive.runVelocity(speeds);
    }

    public void buildAutos(LoggedDashboardChooser<Command> autoChooser) {

        autoChooser.addOption(
                "TestMode",
                new PathFinderCommand(
                                r,
                                () ->
                                        r.drive
                                                .getPose()
                                                .plus(new Transform2d(3, 0, Rotation2d.k180deg)))
                        .andThen(
                                ShooterCommands.smarterShootNoGather(
                                                r, () -> 0, () -> 0, FieldConstants.Hub.center)
                                        .withTimeout(10)
                                        .finallyDo(
                                                () -> {
                                                    r.shooter.stop().execute();
                                                    r.spindexter.stop().execute();
                                                    r.intake.extend();
                                                    r.intake.stopIntake().execute();
                                                })));

        autoChooser.addOption("LeftTrenchDoubleScoop", buildTrenchLeftDoubleScoop());
        autoChooser.addOption("RightTrenchDoubleScoop", buildTrenchRightDoubleScoop());
        autoChooser.addOption("JustShoot", buildSitStillAndShoot());
        autoChooser.addOption("LeftBumpDoubleScoop", buildLeftDoubleScoop());
        autoChooser.addOption("JustDrop", buildJustDrop());
    }

    public Command buildLeftDoubleScoop() {
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        // first drop the intake as fast as possible
        sequence.addCommands(r.intake.fastDrop());
        // drive the profile while intaking
        ParallelDeadlineGroup parallelGroup =
                new ParallelDeadlineGroup(loadTraj("circleleft.traj"), r.intake.smartIntake());
        sequence.addCommands(parallelGroup);
        // shoot the balls while driving to the second start point
        sequence.addCommands(
                ShooterCommands.smarterShootNoGather(r, () -> 0, () -> 0, FieldConstants.Hub.center)
                        .withTimeout(10)
                        .finallyDo(
                                () -> {
                                    r.shooter.stop().execute();
                                    r.spindexter.stop().execute();
                                    r.intake.extend();
                                    r.intake.stopIntake().execute();
                                })
                /*.alongWith(
                DriveCommands.driveToPoint(
                                r,
                                () ->
                                        FieldConstants.flipIfRed(
                                                FieldConstants.Locations
                                                        .trenchLeftStart
                                                        .plus(
                                                                new Transform2d(
                                                                        1.5,
                                                                        0,
                                                                        Rotation2d
                                                                                .kZero))))
                        .andThen(
                                DriveCommands.driveToPoint(
                                        r,
                                        () ->
                                                FieldConstants.flipIfRed(
                                                        FieldConstants.Locations
                                                                .trenchLeftStart))))*/ );
        // drive the second profile while intaking
        parallelGroup =
                new ParallelDeadlineGroup(loadTraj("circleleft_pt2.traj"), r.intake.smartIntake());
        sequence.addCommands(r.intake.fastDrop());
        sequence.addCommands(parallelGroup);
        // shoot again for the remaining time
        sequence.addCommands(
                ShooterCommands.smarterShootNoGather(
                        r, () -> 0, () -> 0, FieldConstants.Hub.center));
        return sequence;
    }

    public Command buildTrenchLeftDoubleScoop() {
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        // first drop the intake as fast as possible
        sequence.addCommands(r.intake.fastDrop());
        // drive the profile while intaking
        ParallelDeadlineGroup parallelGroup =
                new ParallelDeadlineGroup(loadTraj("lefttrench.traj"), r.intake.smartIntake());
        sequence.addCommands(parallelGroup);

        // shoot the balls while stationary
        sequence.addCommands(
                ShooterCommands.smarterShootNoGather(r, () -> 0, () -> 0, FieldConstants.Hub.center)
                        .withTimeout(5)
                        .finallyDo(
                                () -> {
                                    r.shooter.stop().execute();
                                    r.spindexter.stop().execute();
                                    r.intake.extend();
                                    r.intake.stopIntake().execute();
                                }));
        sequence.addCommands(r.intake.fastDrop());

        // drive the second profile while intaking
        parallelGroup =
                new ParallelDeadlineGroup(loadTraj("trenchleft_pt2.traj"), r.intake.smartIntake());
        sequence.addCommands(parallelGroup);

        // shoot again for the remaining time
        sequence.addCommands(
                ShooterCommands.smarterShootNoGather(r, () -> 0, () -> 0, FieldConstants.Hub.center)
                        .withTimeout(5));
        return sequence;
    }

    public Command buildTrenchRightDoubleScoop() {
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        // first drop the intake as fast as possible
        sequence.addCommands(r.intake.fastDrop());
        // drive the profile while intaking
        ParallelDeadlineGroup parallelGroup =
                new ParallelDeadlineGroup(loadTraj("righttrench.traj"), r.intake.smartIntake());
        sequence.addCommands(parallelGroup);

        // shoot the balls while stationary
        sequence.addCommands(
                ShooterCommands.smarterShootNoGather(r, () -> 0, () -> 0, FieldConstants.Hub.center)
                        .withTimeout(5)
                        .finallyDo(
                                () -> {
                                    r.shooter.stop().execute();
                                    r.spindexter.stop().execute();
                                    r.intake.extend();
                                    r.intake.stopIntake().execute();
                                }));
        sequence.addCommands(r.intake.fastDrop());

        // drive the second profile while intaking
        parallelGroup =
                new ParallelDeadlineGroup(loadTraj("trenchright_pt2.traj"), r.intake.smartIntake());
        sequence.addCommands(parallelGroup);

        // shoot again for the remaining time
        sequence.addCommands(
                ShooterCommands.smarterShootNoGather(
                        r, () -> 0, () -> 0, FieldConstants.Hub.center));
        return sequence;
    }

    private Command buildSitStillAndShoot() {
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addCommands(
                ShooterCommands.smarterShootNoGather(r, () -> 0, () -> 0, FieldConstants.Hub.center)
                        .withTimeout(5)
                        .finallyDo(
                                () -> {
                                    r.shooter.stop().execute();
                                    r.spindexter.stop().execute();
                                    r.intake.extend();
                                    r.intake.stopIntake().execute();
                                }));
        return sequence;
    }

    private Command buildShootWithDepot() {
        double offset = Units.inchesToMeters(27);
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addCommands(
                new PathFinderCommand(
                        r, () -> new Pose2d(FieldConstants.Locations.depot, Rotation2d.k180deg)));
        sequence.addCommands(r.intake.fastDrop());
        sequence.addCommands(
                new ParallelDeadlineGroup(
                        DriveCommands.driveToPoint(
                                r,
                                () ->
                                        r.drive
                                                .getPose()
                                                .plus(
                                                        new Transform2d(
                                                                Units.inchesToMeters(
                                                                        offset
                                                                                + Constants
                                                                                                .robotLength
                                                                                        / 2.0),
                                                                0.0,
                                                                Rotation2d.kZero)))));
        return sequence;
    }

    private Command buildJustDrop() {
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addCommands(r.intake.fastDrop());
        return sequence;
    }
}
