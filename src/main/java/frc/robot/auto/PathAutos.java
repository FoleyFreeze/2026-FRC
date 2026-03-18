package frc.robot.auto;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.PathFinderCommand;

public class PathAutos {
    private RobotContainer r;
    public PathPlannerPath rightSideToNeutralZone = loadPath("");

    public PathAutos(RobotContainer r){
        this.r = r;
    }

    private PathPlannerPath loadPath(String name){
        try{
            return PathPlannerPath.fromPathFile(name);
        } catch (Exception e){
            System.out.println("Big oops:");
            System.out.println("Loading of path: " + name + " has failed:");
            e.printStackTrace();

            return new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(new Pose2d()), 
                new PathConstraints(0, 0, 0, 0), 
                null, 
                new GoalEndState(0, Rotation2d.kZero)
            );
        }
    }
    
    public void buildAutos(LoggedDashboardChooser<Command> autoChooser){
        autoChooser.addOption("null", pathFunc1());
    }

    public Pose2d pose1Default = new Pose2d(8.193029403686523,0.7835342288017273, new Rotation2d(1.5253724861748186));

public Supplier<Pose2d> poseMaker(double x, double y, double theta){
    return () -> new Pose2d(
        new Translation2d(x, y),
        new Rotation2d(theta)
    );
}

    public Command pathFunc1(){
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addCommands(r.intake.fastDrop());
                sequence.addCommands(
                new PathFinderCommand(r,poseMaker(8.193029403686523, 0.7835342288017273, 1.5253724861748186)));
        //TODO: add rest of points from the choreo auto "TEST AUTO LEFT"
        return sequence;
    }
}
