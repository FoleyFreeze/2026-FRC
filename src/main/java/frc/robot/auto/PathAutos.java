package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    // map some runnable with each auton. Using these to zero the robot to the auton start as soon
    // as its selected
    // so that any position error can be learned from there by vision between when auton is selected
    // and when auton starts
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
        autoChooser.addOption("RightTrenchTwoScoop", rightTrenchOutsideLoop());
        autoChooser.addOption("LeftBumpTwoScoop", leftBumpOutside());
        autoChooser.addOption("RightBumpTwoScop", rightBumpOutside());
        autoChooser.addOption("SitStillAndShoot", buildSitStillAndShoot());
    }

    private Command buildSitStillAndShoot() {
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addCommands(
                ShooterCommands.smartShoot(r, FieldConstants.Hub.center)
                        .withTimeout(5)
                        .finallyDo(
                                () -> {
                                    r.shooter.stopAll().execute();
                                    r.spindexter.stop().execute();
                                    r.intake.extend();
                                    r.intake.stopIntake().execute();
                                }));
        return sequence;
    }

    public Supplier<Pose2d> poseMaker(double x, double y, double theta) {
        return () -> new Pose2d(new Translation2d(x, y), new Rotation2d(theta));
    }

    public Command leftTrenchOutsideLoop() {
        Command auto = twoScoopAuto(leftSideToNeutralZone, leftSideTrenchToInside);
        Runnable run =
                () -> {
                    Rotation2d rot = leftSideToNeutralZone.getIdealStartingState().rotation();
                    Translation2d tx = leftSideToNeutralZone.getPoint(0).position;
                    Pose2d pose = new Pose2d(tx, rot);
                    r.drive.setPose(FieldConstants.flipIfRed(pose));
                };
        pathMap.put(auto, run);
        return auto;
    }

    public Command rightTrenchOutsideLoop() {
        Command auto = twoScoopAuto(rightSideToNeutralZone, rightSideTrenchToInside);
        Runnable run =
                () -> {
                    Rotation2d rot = rightSideToNeutralZone.getIdealStartingState().rotation();
                    Translation2d tx = rightSideToNeutralZone.getPoint(0).position;
                    Pose2d pose = new Pose2d(tx, rot);
                    r.drive.setPose(FieldConstants.flipIfRed(pose));
                };
        pathMap.put(auto, run);
        return auto;
    }

    public Command leftBumpOutside() {
        Command auto = twoScoopAuto(leftBumpOutside, leftBumpInside);
        Runnable run =
                () -> {
                    Rotation2d rot = leftBumpOutside.getIdealStartingState().rotation();
                    Translation2d tx = leftBumpOutside.getPoint(0).position;
                    Pose2d pose = new Pose2d(tx, rot);
                    r.drive.setPose(FieldConstants.flipIfRed(pose));
                };
        pathMap.put(auto, run);
        return auto;
    }

    public Command rightBumpOutside() {
        Command auto = twoScoopAuto(rightBumpOutside, rightBumpInside);
        Runnable run =
                () -> {
                    Rotation2d rot = rightBumpOutside.getIdealStartingState().rotation();
                    Translation2d tx = rightBumpOutside.getPoint(0).position;
                    Pose2d pose = new Pose2d(tx, rot);
                    r.drive.setPose(FieldConstants.flipIfRed(pose));
                };
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
                ShooterCommands.smartShoot(r, FieldConstants.Hub.center)
                        .withTimeout(initialShootWait)
                        .finallyDo(
                                () -> {
                                    r.shooter.stopAll().execute();
                                    r.spindexter.stop().execute();
                                    r.intake.extend();
                                    r.intake.stopIntake().execute();
                                }));

        // drive the profile while intaking
        ParallelDeadlineGroup parallelGroup =
                new ParallelDeadlineGroup(
                        AutoBuilder.followPath(path1),
                        r.intake.smartIntake(),
                        r.shooter.pointAtHub());
        sequence.addCommands(parallelGroup);

        // shoot the balls while stationary
        sequence.addCommands(
                ShooterCommands.smartShoot(r, FieldConstants.Hub.center)
                        .withTimeout(firstShootTime)
                        .finallyDo(
                                () -> {
                                    r.shooter.stopAll().execute();
                                    r.spindexter.stop().execute();
                                    r.intake.extend();
                                    r.intake.stopIntake().execute();
                                }));
        sequence.addCommands(r.intake.fastDrop());

        // drive the second profile while intaking
        parallelGroup =
                new ParallelDeadlineGroup(
                        AutoBuilder.followPath(path2),
                        r.intake.smartIntake(),
                        r.shooter.pointAtHub());
        sequence.addCommands(parallelGroup);

        // shoot again for the remaining time
        sequence.addCommands(
                ShooterCommands.smartShoot(r, FieldConstants.Hub.center)
                        .withTimeout(secondShootTime));

        return new ConditionalCommand(abortCommand(), sequence, this::shouldAbort);
    }

    // abort auton if things like the pigeon or limelight are not connected
    private boolean shouldAbort() {
        return false;
    }

    private Command abortCommand() {
        return new InstantCommand();
    }
}
