package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climber.Climber.AutoClimbState;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class ClimbCommands {
    public static Command autoClimb(RobotContainer r, Supplier<Pose2d> climbLoc, double yOffset) {
        Supplier<Pose2d> offsetPose =
                () -> climbLoc.get().plus(new Transform2d(0, yOffset, Rotation2d.kZero));
        SequentialCommandGroup sc = new SequentialCommandGroup();
        sc.addCommands(
                new PathFinderCommand(r, offsetPose)
                        .alongWith(
                                new WaitUntilCommand(() -> getDist(r, offsetPose.get()) < 1.5)
                                        .andThen(r.climber.autoExtendCmd()))
                        .alongWith(setClimbEnum(r, AutoClimbState.PRECLIMB))
                        .unless(() -> getDist(r, offsetPose.get()) < Units.inchesToMeters(9.5)));
        sc.addCommands(
                DriveCommands.driveToPoint(r, () -> climbLoc.get())
                        .alongWith(setClimbEnum(r, AutoClimbState.PATHFIND_DONE))
                        .unless(checkEnum(r, AutoClimbState.POINT_DRIVEN)));
        sc.addCommands(
                r.climber.liftUpBotCmd().alongWith(setClimbEnum(r, AutoClimbState.POINT_DRIVEN)));
        sc.addCommands(setClimbEnum(r, AutoClimbState.ROBOT_LIFTED));
        return sc;
    }

    public static Command autoDown(RobotContainer r) {

        SequentialCommandGroup sc = new SequentialCommandGroup();
        sc.addCommands(r.climber.pushDownBotCmd());
        sc.addCommands(new WaitUntilCommand(() -> getDist(r, r.climber.climbLocDecided) > 0.5));
        // TODO: above
        sc.addCommands(
                r.climber
                        .retractClimberCmd()
                        .alongWith(setClimbEnum(r, AutoClimbState.PRECLIMB))
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
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

    public static Command setClimbEnum(RobotContainer r, AutoClimbState enumValue) {
        return new InstantCommand(() -> r.climber.autoClimbState = enumValue);
    }

    public static BooleanSupplier checkEnum(RobotContainer r, AutoClimbState autoClimbState) {
        return () -> r.climber.autoClimbState == autoClimbState;
    }
}
