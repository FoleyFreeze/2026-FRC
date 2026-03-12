package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PathFinderCommand;
import frc.robot.commands.ShooterCommands;
import frc.robot.util.Util2;
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
                        r.drive::setPose, // A function that resets the current robot pose to the
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
        Logger.recordOutput("Choreo/Xerr", pose.getX() - sample.getPose().getX());
        Logger.recordOutput("Choreo/Yerr", pose.getY() - sample.getPose().getY());
        Logger.recordOutput(
                "Choreo/Rerr", pose.getRotation().minus(sample.getPose().getRotation()));

        // Generate the next speeds for the robot
        ChassisSpeeds speeds =
                new ChassisSpeeds(
                        sample.vx + xController.calculate(pose.getX(), sample.x),
                        sample.vy + yController.calculate(pose.getY(), sample.y),
                        sample.omega
                                + headingController.calculate(
                                        pose.getRotation().getRadians(), sample.heading));

        // Apply the generated speeds
        speeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        r.drive.getRotation());
        r.drive.runVelocity(speeds);
    }

    public void buildAutos(LoggedDashboardChooser<Command> autoChooser) {

        autoChooser.addOption(
                "TestMode",
                new PathFinderCommand(
                        r,
                        () -> r.drive.getPose().plus(new Transform2d(3, 0, Rotation2d.k180deg))));
        autoChooser.addOption("LeftDoubleScoopBump", buildLeftDoubleScoop());
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
                ShooterCommands.smarterShootNoGather(r, null, FieldConstants.Hub.center)
                        .withTimeout(10)
                        .alongWith(
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
                                                                                .trenchLeftStart)))));
        // drive the second profile while intaking
        parallelGroup =
                new ParallelDeadlineGroup(loadTraj("circleleft2.traj"), r.intake.smartIntake());
        sequence.addCommands(r.intake.fastDrop());
        sequence.addCommands(parallelGroup);
        // shoot again for the remaining time
        sequence.addCommands(
                ShooterCommands.smarterShootNoGather(r, null, FieldConstants.Hub.center));
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
        // shoot the balls while driving to the second start point
        sequence.addCommands(
                ShooterCommands.smarterShootNoGather(r, null, FieldConstants.Hub.center)
                        .withTimeout(10));
        sequence.addCommands(r.intake.fastDrop());

        sequence.addCommands(DriveCommands.driveToPoint(r,() ->FieldConstants.flipIfRed(FieldConstants.Locations.trenchLeftStart)));
        // drive the second profile while intaking
        parallelGroup =
                new ParallelDeadlineGroup(loadTraj("circleleft2.traj"), r.intake.smartIntake());
        sequence.addCommands(parallelGroup);
        // shoot again for the remaining time
        sequence.addCommands(
                ShooterCommands.smarterShootNoGather(r, null, FieldConstants.Hub.center));
        return sequence;
    }
}
