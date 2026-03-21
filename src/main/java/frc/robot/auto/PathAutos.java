package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.ShooterCommands;
import java.util.HashMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class PathAutos {
    private RobotContainer r;
    public PathPlannerPath leftSideToNeutralZone = loadPath("LeftTrenchOutsideLoop");
    public PathPlannerPath rightSideToNeutralZone = leftSideToNeutralZone.mirrorPath();
    public PathPlannerPath leftSideTrenchToInside = loadPath("LeftTrenchInsideLoop");
    public PathPlannerPath rightSideTrenchToInside = leftSideTrenchToInside.mirrorPath();
    public PathPlannerPath leftDefenceDownCenter = loadPath("DefenceDownCenter");
    public PathPlannerPath rightDefenceDownCenter = leftDefenceDownCenter.mirrorPath();
    public PathPlannerPath leftBumpOutside = loadPath("LeftBumpOutsideLoop");
    public PathPlannerPath rightBumpOutside = leftBumpOutside.mirrorPath();
    public PathPlannerPath leftBumpInside = loadPath("LeftBumpInsideLoop");
    public PathPlannerPath rightBumpInside = leftBumpInside.mirrorPath();

    public HashMap<Command, Runnable> pathMap = new HashMap<>();

    public PathAutos(RobotContainer r) {
        this.r = r;
    }

    private PathPlannerPath loadPath(String name) {
        try {
            return PathPlannerPath.fromPathFile(name);
        } catch (Exception e) {
            System.out.println("Big oops:");
            System.out.println("Loading of path: " + name + " has failed:");
            e.printStackTrace();

            return new PathPlannerPath(
                    PathPlannerPath.waypointsFromPoses(new Pose2d()),
                    new PathConstraints(0, 0, 0, 0),
                    null,
                    new GoalEndState(0, Rotation2d.kZero));
        }
    }

    public void buildAutos(LoggedDashboardChooser<Command> autoChooser) {
        autoChooser.addOption("leftTrenchTwoScoop", leftTrenchOutsideLoop());
        autoChooser.addOption("RightTrenchOutside", rightTrenchOutsideLoop());
        autoChooser.addOption("LeftBumpTwoScoop", leftBumpOutside());
        autoChooser.addOption("RightBumpTwoScop", rightBumpOutside());
    }

    public Supplier<Pose2d> poseMaker(double x, double y, double theta) {
        return () -> new Pose2d(new Translation2d(x, y), new Rotation2d(theta));
    }

    public Command leftTrenchOutsideLoop() {
        Command auto = twoScoopAuto(leftSideToNeutralZone, leftSideTrenchToInside);
        Runnable run =
                () -> r.drive.setRotation(leftSideToNeutralZone.getIdealStartingState().rotation());
        pathMap.put(auto, run);
        return auto;
    }

    public Command rightTrenchOutsideLoop() {
        Command auto = twoScoopAuto(rightSideToNeutralZone, rightSideTrenchToInside);
        Runnable run =
                () ->
                        r.drive.setRotation(
                                rightSideToNeutralZone.getIdealStartingState().rotation());
        pathMap.put(auto, run);
        return auto;
    }

    public Command leftBumpOutside() {
        Command auto = twoScoopAuto(leftBumpOutside, leftBumpInside);
        Runnable run =
                () -> r.drive.setRotation(leftBumpOutside.getIdealStartingState().rotation());
        pathMap.put(auto, run);
        return auto;
    }

    public Command rightBumpOutside() {
        Command auto = twoScoopAuto(rightBumpOutside, rightBumpInside);
        Runnable run =
                () -> r.drive.setRotation(rightBumpOutside.getIdealStartingState().rotation());
        pathMap.put(auto, run);
        return auto;
    }

    public Command twoScoopAuto(PathPlannerPath path1, PathPlannerPath path2) {
        double initialShootWait = 1.2;
        double firstShootTime = 3.8;
        double secondShootTime = 5;

        SequentialCommandGroup sequence = new SequentialCommandGroup();
        // first drop the intake as fast as possible
        sequence.addCommands(
                ShooterCommands.smarterShootAndGather(
                                r, () -> 0, () -> 0, FieldConstants.Hub.center)
                        .withTimeout(initialShootWait)
                        .finallyDo(
                                () -> {
                                    r.shooter.stop().execute();
                                    r.spindexter.stop().execute();
                                    r.intake.extend();
                                    r.intake.stopIntake().execute();
                                }));

        // drive the profile while intaking
        ParallelDeadlineGroup parallelGroup =
                new ParallelDeadlineGroup(AutoBuilder.followPath(path1), r.intake.smartIntake());
        sequence.addCommands(parallelGroup);

        // shoot the balls while stationary
        sequence.addCommands(
                ShooterCommands.smarterShootNoGather(r, () -> 0, () -> 0, FieldConstants.Hub.center)
                        .withTimeout(firstShootTime)
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
                new ParallelDeadlineGroup(AutoBuilder.followPath(path2), r.intake.smartIntake());
        sequence.addCommands(parallelGroup);

        // shoot again for the remaining time
        sequence.addCommands(
                ShooterCommands.smarterShootNoGather(r, () -> 0, () -> 0, FieldConstants.Hub.center)
                        .withTimeout(secondShootTime));
        return sequence;
    }
}
