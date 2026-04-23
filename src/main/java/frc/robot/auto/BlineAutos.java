package frc.robot.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.lib.BLine.FollowPath;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class BlineAutos {

    RobotContainer r;
    FollowPath.Builder pathBuilder;

    public BlineAutos(RobotContainer r) {
        this.r = r;
        pathBuilder =
                new FollowPath.Builder(
                                r.drive, // Subsystem requirement
                                r.drive::getPose, // Supplier<Pose2d>
                                r.drive::getChassisSpeeds, // Supplier<ChassisSpeeds>
                                // (robot-relative)
                                r.drive::runVelocity, // Consumer<ChassisSpeeds>  (robot-relative)
                                new PIDController(
                                        5.0, 0.0,
                                        0.0), // translation — minimizes remaining distance
                                new PIDController(
                                        3.0, 0.0, 0.0), // rotation    — minimizes heading error
                                new PIDController(
                                        2.0, 0.0,
                                        0.0) // cross-track — minimizes perpendicular deviation
                                )
                        .withDefaultShouldFlip(); // auto-flip when on the red alliance
        // .withPoseReset(r.drive::resetPose); // reset odometry at each path's start pose

        FollowPath.setDoubleLoggingConsumer(p -> Logger.recordOutput(p.getFirst(), p.getSecond()));
        FollowPath.setBooleanLoggingConsumer(p -> Logger.recordOutput(p.getFirst(), p.getSecond()));
        FollowPath.setPoseLoggingConsumer(p -> Logger.recordOutput(p.getFirst(), p.getSecond()));
        FollowPath.setTranslationListLoggingConsumer(
                p -> Logger.recordOutput(p.getFirst(), p.getSecond()));
    }

    public void buildAutos(LoggedDashboardChooser<Command> autoChooser) {}
}
