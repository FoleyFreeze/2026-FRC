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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommands;
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
    public PathPlannerPath middleLeftMiddle1 = loadPath("MiddleLeftMiddle1");
    public PathPlannerPath middleRightMiddle1 = middleLeftMiddle1.mirrorPath();
    public PathPlannerPath middleLeftMiddle2 = loadPath("MiddleLeftMiddle2");
    public PathPlannerPath middleRightMiddle2 = middleLeftMiddle2.mirrorPath();
    public PathPlannerPath forwardLeftBump1 = loadPath("ForwardLeftBump1");
    public PathPlannerPath forwardLeftBump2 = loadPath("ForwardLeftBump2");
    public PathPlannerPath forwardRightBump1 = forwardLeftBump1.mirrorPath();
    public PathPlannerPath forwardRightBump2 = forwardLeftBump2.mirrorPath();

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
        autoChooser.addOption("ForwardLeftBump", buildFwdLeftBump());
        autoChooser.addOption("ForwardRightBump", buildFwdRightBump());
        autoChooser.addOption("MiddleLeftDepot", buildMiddleLeftDepot());
        autoChooser.addOption("MiddleRightDepot", buildMiddleRightDepot());
        autoChooser.addOption("MiddleLeftBump", buildMiddleLeftMiddle());
        autoChooser.addOption("MiddleRightBump", buildMiddleRightMiddle());
        autoChooser.addOption("LeftBumpTwoScoop", leftBumpOutside());
        autoChooser.addOption("RightBumpTwoScop", rightBumpOutside());
        autoChooser.addOption("SitStillAndShoot", buildSitStillAndShoot());
    }

    private Command buildMiddleLeftDepot() {
        Command auto = singleScoopDepot(middleLeftMiddle1);
        Runnable run =
                () -> {
                    Rotation2d rot = middleLeftMiddle1.getIdealStartingState().rotation();
                    Translation2d tx = middleLeftMiddle1.getPoint(0).position;
                    Pose2d pose = new Pose2d(tx, rot);
                    r.drive.setPose(FieldConstants.flipIfRed(pose));
                };
        pathMap.put(auto, run);
        return auto;
    }

    private Command buildMiddleRightDepot() {
        Command auto = singleScoopDepot(middleRightMiddle1);
        Runnable run =
                () -> {
                    Rotation2d rot = middleRightMiddle1.getIdealStartingState().rotation();
                    Translation2d tx = middleRightMiddle1.getPoint(0).position;
                    Pose2d pose = new Pose2d(tx, rot);
                    r.drive.setPose(FieldConstants.flipIfRed(pose));
                };
        pathMap.put(auto, run);
        return auto;
    }

    private Command buildMiddleLeftMiddle() {
        Command auto = twoMiddleScoopAuto(middleLeftMiddle1, middleLeftMiddle2);
        Runnable run =
                () -> {
                    Rotation2d rot = middleLeftMiddle1.getIdealStartingState().rotation();
                    Translation2d tx = middleLeftMiddle1.getPoint(0).position;
                    Pose2d pose = new Pose2d(tx, rot);
                    r.drive.setPose(FieldConstants.flipIfRed(pose));
                };
        pathMap.put(auto, run);
        return auto;
    }

    private Command buildMiddleRightMiddle() {
        Command auto = twoMiddleScoopAuto(middleRightMiddle1, middleRightMiddle2);
        Runnable run =
                () -> {
                    Rotation2d rot = middleRightMiddle1.getIdealStartingState().rotation();
                    Translation2d tx = middleRightMiddle1.getPoint(0).position;
                    Pose2d pose = new Pose2d(tx, rot);
                    r.drive.setPose(FieldConstants.flipIfRed(pose));
                };
        pathMap.put(auto, run);
        return auto;
    }

    private Command buildFwdLeftBump() {
        Command auto = twoScoopAuto(forwardLeftBump1, forwardLeftBump2);
        Runnable run =
                () -> {
                    Rotation2d rot = forwardLeftBump1.getIdealStartingState().rotation();
                    Translation2d tx = forwardLeftBump1.getPoint(0).position;
                    Pose2d pose = new Pose2d(tx, rot);
                    r.drive.setPose(FieldConstants.flipIfRed(pose));
                };
        pathMap.put(auto, run);
        return auto;
    }

    private Command buildFwdRightBump() {
        Command auto = twoScoopAuto(forwardRightBump1, forwardRightBump2);
        Runnable run =
                () -> {
                    Rotation2d rot = forwardRightBump1.getIdealStartingState().rotation();
                    Translation2d tx = forwardRightBump1.getPoint(0).position;
                    Pose2d pose = new Pose2d(tx, rot);
                    r.drive.setPose(FieldConstants.flipIfRed(pose));
                };
        pathMap.put(auto, run);
        return auto;
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
        double initialShootWait = 1.6;
        double firstShootTime = 5;
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

        // shoot the balls while potentially moving
        Pose2d nextPathStart =
                new Pose2d(path2.getPoint(0).position, path2.getIdealStartingState().rotation());
        PathConstraints moveAndShootLimits = new PathConstraints(0.75, 0.75, 1, 1);
        sequence.addCommands(
                ShooterCommands.smartShoot(r, FieldConstants.Hub.center)
                        .alongWith(r.intake.shakeTheIntake())
                        .withTimeout(firstShootTime)
                        .finallyDo(
                                () -> {
                                    r.shooter.stopAll().execute();
                                    r.spindexter.stop().execute();
                                    r.intake.extend();
                                    r.intake.stopIntake().execute();
                                })
                        .alongWith(
                                AutoBuilder.pathfindToPoseFlipped(
                                        nextPathStart, moveAndShootLimits)));
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

    public Command twoMiddleScoopAuto(PathPlannerPath path1, PathPlannerPath path2) {
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

        // shoot the balls while potentially moving
        Pose2d nextPathStart =
                new Pose2d(path2.getPoint(0).position, path2.getIdealStartingState().rotation());
        PathConstraints moveAndShootLimits = new PathConstraints(0.75, 0.75, 1, 1);
        sequence.addCommands(
                ShooterCommands.smartShoot(r, FieldConstants.Hub.center)
                        .alongWith(r.intake.shakeTheIntake())
                        .withTimeout(firstShootTime)
                        .finallyDo(
                                () -> {
                                    r.shooter.stopAll().execute();
                                    r.spindexter.stop().execute();
                                    r.intake.extend();
                                    r.intake.stopIntake().execute();
                                })
                        .alongWith(
                                AutoBuilder.pathfindToPoseFlipped(
                                        nextPathStart, moveAndShootLimits))
                        .andThen(
                                DriveCommands.driveToPoint(
                                        r, () -> FieldConstants.flipIfRed(nextPathStart))));
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

    public Command singleScoopDepot(PathPlannerPath path) {
        double initialShootWait = 1.1;
        double firstShootTime = 5;
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
                        AutoBuilder.followPath(path),
                        r.intake.smartIntake(),
                        r.shooter.pointAtHub());
        sequence.addCommands(parallelGroup);

        // shoot the balls while potentially moving
        Pose2d nextPathStart = new Pose2d(FieldConstants.Depot.inFrontOfDepot, Rotation2d.k180deg);
        PathConstraints moveAndShootLimits = new PathConstraints(0.75, 0.75, 1, 1);
        sequence.addCommands(
                ShooterCommands.smartShoot(r, FieldConstants.Hub.center)
                        .alongWith(r.intake.shakeTheIntake())
                        .withTimeout(firstShootTime)
                        .finallyDo(
                                () -> {
                                    r.shooter.stopAll().execute();
                                    r.spindexter.stop().execute();
                                    r.intake.extend();
                                    r.intake.stopIntake().execute();
                                })
                        .alongWith(
                                AutoBuilder.pathfindToPoseFlipped(
                                        nextPathStart, moveAndShootLimits))
                        .andThen(
                                DriveCommands.driveToPoint(
                                        r, () -> FieldConstants.flipIfRed(nextPathStart))));
        sequence.addCommands(r.intake.fastDrop());

        // drive the second profile while intaking
        parallelGroup =
                new ParallelDeadlineGroup(
                        DriveCommands.driveToPoint(
                                        r,
                                        () ->
                                                FieldConstants.flipIfRed(
                                                        new Pose2d(
                                                                FieldConstants.Depot.depotWall,
                                                                Rotation2d.k180deg)))
                                .andThen(new WaitCommand(secondShootTime)),
                        r.intake.smartIntake(),
                        ShooterCommands.smartShoot(r, FieldConstants.Hub.center));
        sequence.addCommands(parallelGroup);

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
