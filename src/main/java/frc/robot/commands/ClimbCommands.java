package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import java.util.function.Supplier;

public class ClimbCommands {
    public static Command autoClimb(RobotContainer r, Supplier<Pose2d> climbLoc, double yOffset) {
        Supplier<Pose2d> offsetPose = () -> climbLoc.get().plus(new Transform2d(0, yOffset, Rotation2d.kZero));
        SequentialCommandGroup sc = new SequentialCommandGroup();
        sc.addCommands(
                new PathFinderCommand(r, offsetPose)
                        .alongWith(
                                new WaitUntilCommand(() -> getDist(r, offsetPose.get()) < 1.5)
                                        .andThen(r.climber.autoExtendCmd())));
        sc.addCommands(DriveCommands.driveToPoint(r, () -> climbLoc.get()));
        sc.addCommands(r.climber.liftUpBotCmd());
        return sc;
    }

    public static Command autoDown(RobotContainer r) {

        SequentialCommandGroup sc = new SequentialCommandGroup();
        sc.addCommands(r.climber.pushDownBotCmd());
        sc.addCommands(new WaitUntilCommand(() -> getDist(r, r.climber.climbLocDecided) > 0.5));
        // TODO: above
        sc.addCommands(r.climber.retractClimberCmd());
        return sc;
    }

    public static double getDist(RobotContainer r, Pose2d pose) {
        
        return r.drive.getPose().getTranslation().minus(pose.getTranslation()).getNorm();
    }

    public static Pose2d determineClimbLoc(
            RobotContainer r) { // TODO: fix this, it doesnt change once set
        if (r.drive.getPose().getY() < FieldConstants.fieldWidth / 2) {
            r.climber.left();
        } else {
            r.climber.right();
        }
        return r.climber.climbLocDecided;
    }
}
