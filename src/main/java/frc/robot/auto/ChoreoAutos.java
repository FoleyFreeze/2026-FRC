package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

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
                        Util.isRedAlliance()
                                ? r.drive.getRotation().plus(new Rotation2d(Math.PI))
                                : r.drive.getRotation());
        r.drive.runVelocity(speeds);
    }
}
