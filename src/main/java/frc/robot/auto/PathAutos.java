package frc.robot.auto;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
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
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.LeftTrench;
import frc.robot.commands.PathFinderCommand;
import frc.robot.commands.ShooterCommands;

public class PathAutos {
    private RobotContainer r;
    public PathPlannerPath leftSideToNeutralZone = loadPath("LeftTrenchOutsideLoop");
    public PathPlannerPath  rightSideToNeutralZone = leftSideToNeutralZone.flipPath(); 
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
        autoChooser.addOption("leftTrenchOneScoop", leftTrenchOutsideLoop());
    }

public Supplier<Pose2d> poseMaker(double x, double y, double theta){
    return () -> new Pose2d(
        new Translation2d(x, y),
        new Rotation2d(theta)
    );
}
    public Command leftTrenchOutsideLoop(){
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addCommands(
                ShooterCommands.smarterShootAndGather(
                                r, () -> 0, () -> 0, FieldConstants.Hub.center)
                        .withTimeout(1.2)
                        .finallyDo(
                                () -> {
                                    r.shooter.stop().execute();
                                    r.spindexter.stop().execute();
                                    r.intake.extend();
                                    r.intake.stopIntake().execute();
                                }));
        sequence.addCommands(AutoBuilder.followPath(leftSideToNeutralZone).alongWith(r.intake.smartIntake()));
        return sequence;
    }
}
