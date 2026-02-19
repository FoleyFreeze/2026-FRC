package frc.robot.commands;

import java.time.OffsetTime;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;

public class ClimbCommands {
    public static Command autoClimb(RobotContainer r, Pose2d climbLoc, double yOffset){
        Pose2d offsetPose = climbLoc.plus(new Transform2d(0,yOffset,Rotation2d.kZero));
        SequentialCommandGroup sc = new SequentialCommandGroup();
        sc.addCommands(new PathFinderCommand(r, ()-> offsetPose)
                .alongWith(new WaitUntilCommand(()-> getDist(r,offsetPose)< 1.5).andThen(r.climber.autoExtendCmd())));
        sc.addCommands(DriveCommands.driveToPoint(r, ()-> climbLoc));
        sc.addCommands(r.climber.liftUpBotCmd());
        return sc;
    }
    public static Command autoDown(RobotContainer r){
        
        SequentialCommandGroup sc = new SequentialCommandGroup();
        sc.addCommands(r.climber.pushDownBotCmd());
        // sc.addCommands(new FunctionalCommand(() -> getDist(r, point) > 0.5)); 
        //TODO: above
        sc.addCommands(r.climber.retractClimberCmd());
        return sc;
    }
    public static double getDist(RobotContainer r, Pose2d pose){
        return r.drive.getPose().getTranslation().minus(pose.getTranslation()).getNorm();
    }

}
