package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.CameraBallGatherCmd;
import frc.robot.commands.CloseBallGatherCmd;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Util2;
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
    public PathPlannerPath neutralPassLeft = loadPath("NeutralPass");
    public PathPlannerPath neutralPassRight = neutralPassLeft.mirrorPath();
    public PathPlannerPath leftSideNibble = loadPath("LeftTrenchNibble");
    public PathPlannerPath rightSideNibble = leftSideNibble.mirrorPath();
    public PathPlannerPath leftCamStart = loadPath("StartCamLeft");
    public PathPlannerPath rightCamStart = leftCamStart.mirrorPath();
    public PathPlannerPath leftCamEnd = loadPath("EndCamLeft");
    public PathPlannerPath rightCamEnd = leftCamEnd.mirrorPath();
    public PathPlannerPath depotPath = loadPath("DriveToDepot");

    double prePrimeTime = 0.8;

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
        // autoChooser.addOption("TestShake", robotShake());
        autoChooser.addOption("leftTrenchTwoNibble", leftTrenchNibble());
        autoChooser.addOption("rightTrenchTwoNibble", rightTrenchNibble());
        autoChooser.addOption("leftTrenchTwoScoop", leftTrenchOutsideLoop());
        autoChooser.addOption("RightTrenchTwoScoop", rightTrenchOutsideLoop());
        autoChooser.addOption("ForwardLeftBump", buildFwdLeftBump());
        autoChooser.addOption("ForwardRightBump", buildFwdRightBump());
        autoChooser.addOption("MiddleLeftDepot", buildMiddleLeftDepot());
        autoChooser.addOption("MiddleRightDepot", buildMiddleRightDepot());
        autoChooser.addOption("JustDepot", buildJustDepot());
        autoChooser.addOption("MiddleLeftBump", buildMiddleLeftMiddle());
        autoChooser.addOption("MiddleRightBump", buildMiddleRightMiddle());
        autoChooser.addOption("LeftBumpTwoScoop", leftBumpOutside());
        autoChooser.addOption("RightBumpTwoScop", rightBumpOutside());
        autoChooser.addOption("SitStillAndShoot", buildSitStillAndShoot());
        autoChooser.addOption("LeftPassNeutral", buildPassNeutralLeft());
        autoChooser.addOption("RightPassNeutral", buildPassNeutralRight());
        autoChooser.addOption("LeftCamAuto", buildCamGatherPathLeft());
        autoChooser.addOption("RightCamAuto", buildCamGatherPathRight());
    }

    private Command buildJustDepot() {
        double initialShootWait = 3.0;

        SequentialCommandGroup sequence = new SequentialCommandGroup();
        // first drop the intake as fast as possible
        sequence.addCommands(
                ShooterCommands.smartShoot(r, FieldConstants.Hub.center)
                        .alongWith(new InstantCommand(r.intake::extend, r.intake))
                        .withTimeout(initialShootWait)
                        .finallyDo(
                                () -> {
                                    r.shooter.stopAll().execute();
                                    r.spindexter.stop().execute();
                                    r.intake.extendToAvoidNet();
                                    r.intake.stopIntake().initialize();
                                }));

        // drive the profile while intaking
        ParallelDeadlineGroup parallelGroup =
                new ParallelDeadlineGroup(
                        AutoBuilder.followPath(depotPath), r.shooter.pointAtHub());
        sequence.addCommands(parallelGroup);

        Pose2d preDepotPose = new Pose2d(0.74, 4.7, new Rotation2d());
        Pose2d depotPose = new Pose2d(0.74, 7.0, new Rotation2d());
        Pose2d postDepotPose = new Pose2d(0.70, 7.5, new Rotation2d());
        Pose2d finalShootPose = new Pose2d(1.67, 6.77, new Rotation2d());

        // back up slightly
        sequence.addCommands(
                DriveCommands.driveToPoint(
                        r,
                        () -> FieldConstants.flipIfRed(preDepotPose),
                        1,
                        () -> FieldConstants.flipIfRed(Rotation2d.fromDegrees(108)),
                        0.75));

        // drop the intake
        sequence.addCommands(new InstantCommand(r.intake::extend), new WaitCommand(1.5));

        // drive over the depot
        parallelGroup =
                new ParallelDeadlineGroup(
                        DriveCommands.driveToPoint(
                                r,
                                () -> FieldConstants.flipIfRed(depotPose),
                                5,
                                () ->
                                        Util2.isRedAlliance()
                                                ? Rotation2d.fromDegrees(-72)
                                                : Rotation2d.fromDegrees(108),
                                1.0),
                        ShooterCommands.smartShoot(r, FieldConstants.Hub.center),
                        r.intake.smartIntake());
        sequence.addCommands(parallelGroup);

        // drive into the wall
        sequence.addCommands(
                DriveCommands.driveToPoint(
                                r,
                                () -> FieldConstants.flipIfRed(postDepotPose),
                                2,
                                () -> FieldConstants.flipIfRed(Rotation2d.kCCW_90deg))
                        .alongWith(r.intake.smartIntake()));

        // backup and shoot
        sequence.addCommands(
                DriveCommands.driveToPoint(
                        r,
                        () -> FieldConstants.flipIfRed(finalShootPose),
                        3,
                        () -> FieldConstants.flipIfRed(Rotation2d.kCCW_90deg)));

        // shoot any last balls
        sequence.addCommands(
                ShooterCommands.smartShoot(r, FieldConstants.Hub.center)
                        .alongWith(r.intake.shakeTheIntake())
                        .raceWith(new WaitCommand(5)));

        sequence.addCommands(
                new InstantCommand(
                        () -> {
                            r.shooter.stopAll().execute();
                            r.spindexter.stop().execute();
                            r.intake.extend();
                            r.intake.stopIntake().initialize();
                        }));

        Runnable run =
                () -> {
                    Rotation2d rot = depotPath.getIdealStartingState().rotation();
                    Translation2d tx = depotPath.getPoint(0).position;
                    Pose2d pose = new Pose2d(tx, rot);
                    r.drive.setPose(FieldConstants.flipIfRed(pose));
                    if (Robot.isSimulation() && Constants.currentMode == Mode.SIM) {
                        r.driveSimulation.setSimulationWorldPose(FieldConstants.flipIfRed(pose));
                    }
                };
        pathMap.put(sequence, run);
        return sequence;
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

    private Command buildPassNeutralLeft() {
        Command auto = singlePathPass(neutralPassLeft);
        Runnable run =
                () -> {
                    Rotation2d rot = neutralPassLeft.getIdealStartingState().rotation();
                    Translation2d tx = neutralPassLeft.getPoint(0).position;
                    Pose2d pose = new Pose2d(tx, rot);
                    r.drive.setPose(FieldConstants.flipIfRed(pose));
                };
        pathMap.put(auto, run);
        return auto;
    }

    private Command buildPassNeutralRight() {
        Command auto = singlePathPass(neutralPassRight);
        Runnable run =
                () -> {
                    Rotation2d rot = neutralPassRight.getIdealStartingState().rotation();
                    Translation2d tx = neutralPassRight.getPoint(0).position;
                    Pose2d pose = new Pose2d(tx, rot);
                    r.drive.setPose(FieldConstants.flipIfRed(pose));
                };
        pathMap.put(auto, run);
        return auto;
    }

    private Command buildCamGatherPathLeft() {
        Command auto = twoMiddleScoopAuto(leftCamStart, leftCamEnd);
        Runnable run =
                () -> {
                    Rotation2d rot = leftCamStart.getIdealStartingState().rotation();
                    Translation2d tx = leftCamEnd.getPoint(0).position;
                    Pose2d pose = new Pose2d(tx, rot);
                    r.drive.setPose(FieldConstants.flipIfRed(pose));
                };
        pathMap.put(auto, run);
        return auto;
    }

    private Command buildCamGatherPathRight() {
        Command auto = twoMiddleScoopAuto(rightCamStart, rightCamEnd);
        Runnable run =
                () -> {
                    Rotation2d rot = rightCamStart.getIdealStartingState().rotation();
                    Translation2d tx = rightCamEnd.getPoint(0).position;
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
                                    r.intake.reallyExtend();
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

    public Command leftTrenchNibble() {
        Command auto = twoScoopAuto(leftSideNibble, leftSideTrenchToInside);
        Runnable run =
                () -> {
                    Rotation2d rot = leftSideNibble.getIdealStartingState().rotation();
                    Translation2d tx = leftSideNibble.getPoint(0).position;
                    Pose2d pose = new Pose2d(tx, rot);
                    r.drive.setPose(FieldConstants.flipIfRed(pose));
                };
        pathMap.put(auto, run);
        return auto;
    }

    public Command rightTrenchNibble() {
        Command auto = twoScoopAuto(rightSideNibble, rightSideTrenchToInside);
        Runnable run =
                () -> {
                    Rotation2d rot = rightSideNibble.getIdealStartingState().rotation();
                    Translation2d tx = rightSideNibble.getPoint(0).position;
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
        double firstShootTime = 10;
        double secondShootTime = 10;

        SequentialCommandGroup sequence = new SequentialCommandGroup();

        sequence.addCommands(r.blineAutos.dynamicWait());

        // first drop the intake as fast as possible
        sequence.addCommands(
                ShooterCommands.smartShoot(r, FieldConstants.Hub.center)
                        .alongWith(new InstantCommand(r.intake::extend, r.intake))
                        .withTimeout(initialShootWait)
                        .finallyDo(
                                () -> {
                                    r.shooter.stopAll().execute();
                                    r.spindexter.stop().execute();
                                    r.intake.reallyExtend();
                                    r.intake.stopIntake().initialize();
                                }));

        Pose2d nextPathStart =
                new Pose2d(path2.getPoint(0).position, path2.getIdealStartingState().rotation());

        Pose2d endPose;
        double endTime;
        try {
            var end = path1.getIdealTrajectory(Drive.PP_CONFIG).get().getEndState();
            endPose = end.pose;
            endTime = end.timeSeconds;
        } catch (Exception e) {
            e.printStackTrace();
            endPose = nextPathStart;
            endTime = 5.5;
        }
        final Pose2d endPose1 = endPose;

        // drive the profile while intaking
        ParallelDeadlineGroup parallelGroup =
                new ParallelDeadlineGroup(
                        AutoBuilder.followPath(path1)
                                .until(() -> r.stats.dt > 0.400)
                                .andThen(
                                        new ConditionalCommand(
                                                AutoBuilder.followPath(path1),
                                                new InstantCommand(),
                                                () -> r.stats.dt > 0.400)),
                        r.intake.smartIntake(),
                        r.shooter
                                .pointAtHub()
                                .withTimeout(endTime - prePrimeTime)
                                .andThen(
                                        new RunCommand(
                                                () ->
                                                        r.shooter.newPrime(
                                                                FieldConstants.Hub.center,
                                                                endPose1,
                                                                true))));
        sequence.addCommands(parallelGroup);

        // shoot the balls while potentially moving

        PathConstraints moveAndShootLimits = new PathConstraints(0.5, 0.5, 1, 1);
        sequence.addCommands(
                new ConditionalCommand(
                                ShooterCommands.smartShoot(r, FieldConstants.Locations.passLeft),
                                ShooterCommands.smartShoot(r, FieldConstants.Hub.center),
                                () -> isInNeutralZone())
                        .alongWith(r.intake.shakeTheIntake())
                        .raceWith(waitForTimeOrNoBalls(firstShootTime))
                        .finallyDo(
                                () -> {
                                    r.shooter.stopAll().execute();
                                    r.spindexter.stop().execute();
                                    r.intake.reallyExtend();
                                    r.intake.stopIntake().initialize();
                                })
                        .alongWith(
                                AutoBuilder.pathfindToPoseFlipped(nextPathStart, moveAndShootLimits)
                                        .andThen(
                                                robotShake()
                                                        .raceWith(
                                                                waitForTimeOrNoBalls(
                                                                        firstShootTime)))));
        sequence.addCommands(r.intake.fastDrop());

        try {
            var end = path2.getIdealTrajectory(Drive.PP_CONFIG).get().getEndState();
            endPose = end.pose;
            endTime = end.timeSeconds;
        } catch (Exception e) {
            e.printStackTrace();
            endPose = nextPathStart;
            endTime = 5.5;
        }
        final Pose2d endPose2 = endPose;

        // drive the second profile while intaking
        parallelGroup =
                new ParallelDeadlineGroup(
                        AutoBuilder.followPath(path2),
                        r.intake.smartIntake(),
                        r.shooter
                                .pointAtHub()
                                .withTimeout(endTime - prePrimeTime)
                                .andThen(
                                        new RunCommand(
                                                () ->
                                                        r.shooter.newPrime(
                                                                FieldConstants.Hub.center,
                                                                endPose2,
                                                                true))));
        sequence.addCommands(parallelGroup);

        // shoot again for the remaining time
        sequence.addCommands(
                new ConditionalCommand(
                                ShooterCommands.smartShoot(r, FieldConstants.Locations.passLeft),
                                ShooterCommands.smartShoot(r, FieldConstants.Hub.center),
                                () -> isInNeutralZone())
                        .alongWith(r.intake.shakeTheIntake())
                        // .raceWith(waitForTimeOrNoBalls(secondShootTime)));
                        .raceWith(new WaitCommand(secondShootTime)));
        sequence.addCommands(r.intake.fastDrop());

        // if somehow theres time left run path 2 again
        // drive the second (third) profile while intaking
        parallelGroup =
                new ParallelDeadlineGroup(
                        AutoBuilder.followPath(path2),
                        r.intake.smartIntake(),
                        r.shooter
                                .pointAtHub()
                                .withTimeout(endTime - prePrimeTime)
                                .andThen(
                                        new RunCommand(
                                                () ->
                                                        r.shooter.newPrime(
                                                                FieldConstants.Hub.center,
                                                                endPose2,
                                                                true))));
        sequence.addCommands(parallelGroup);

        // shoot again for the remaining time
        sequence.addCommands(
                new ConditionalCommand(
                                ShooterCommands.smartShoot(r, FieldConstants.Locations.passLeft),
                                ShooterCommands.smartShoot(r, FieldConstants.Hub.center),
                                () -> isInNeutralZone())
                        .alongWith(r.intake.shakeTheIntake())
                        .raceWith(waitForTimeOrNoBalls(secondShootTime)));

        return new ConditionalCommand(abortCommand(), sequence, this::shouldAbort);
    }

    public Command CamGatherAuto(PathPlannerPath path1, PathPlannerPath path2) {
        double initialShootWait = 1.0;
        double firstShootTime = 4.5;
        double secondShootTime = 5;

        SequentialCommandGroup sequence = new SequentialCommandGroup();
        // first drop the intake as fast as possible
        sequence.addCommands(
                ShooterCommands.smartShoot(r, FieldConstants.Hub.center)
                        .alongWith(new InstantCommand(r.intake::extend, r.intake))
                        .withTimeout(initialShootWait)
                        .finallyDo(
                                () -> {
                                    r.shooter.stopAll().execute();
                                    r.spindexter.stop().execute();
                                    r.intake.reallyExtend();
                                    r.intake.stopIntake().initialize();
                                }));

        Pose2d nextPathStart =
                new Pose2d(path2.getPoint(0).position, path2.getIdealStartingState().rotation());

        Pose2d endPose;
        double endTime;
        try {
            var end = path1.getIdealTrajectory(Drive.PP_CONFIG).get().getEndState();
            endPose = end.pose;
            endTime = end.timeSeconds;
        } catch (Exception e) {
            e.printStackTrace();
            endPose = nextPathStart;
            endTime = 5.5;
        }
        final Pose2d endPose1 = endPose;
        // drive the profile while intaking
        ParallelDeadlineGroup parallelGroup =
                new ParallelDeadlineGroup(
                        new WaitCommand(7 + 3.2), // 7 is guestimate for how long to intake for: 3.2
                        // is another guestimate based off from how long the
                        // start auto takes,
                        AutoBuilder.followPath(path1)
                                .andThen(
                                        new CameraBallGatherCmd(r)
                                                .handleInterrupt(
                                                        (Runnable) new CloseBallGatherCmd(r))),
                        r.intake.smartIntake(),
                        r.shooter
                                .pointAtHub()
                                .withTimeout(endTime - prePrimeTime)
                                .andThen(
                                        new RunCommand(
                                                () ->
                                                        r.shooter.newPrime(
                                                                FieldConstants.Hub.center,
                                                                endPose1,
                                                                true))));
        sequence.addCommands(parallelGroup);
        sequence.addCommands(
                AutoBuilder.pathfindToPoseFlipped(endPose1, path2.getGlobalConstraints()));

        // shoot the balls while potentially moving

        PathConstraints moveAndShootLimits = new PathConstraints(0.75, 0.75, 1, 1);
        sequence.addCommands(
                ShooterCommands.smartShoot(r, FieldConstants.Hub.center)
                        .alongWith(r.intake.shakeTheIntake())
                        .withTimeout(firstShootTime)
                        .finallyDo(
                                () -> {
                                    r.shooter.stopAll().execute();
                                    r.spindexter.stop().execute();
                                    r.intake.reallyExtend();
                                    r.intake.stopIntake().initialize();
                                })
                        .alongWith(
                                AutoBuilder.pathfindToPoseFlipped(
                                        nextPathStart, moveAndShootLimits)));
        // sequence.addCommands(r.intake.fastDrop());

        // drive the second profile while intaking
        parallelGroup =
                new ParallelDeadlineGroup(
                        new WaitCommand(8 + 3.2), // 8 is guestimate for how long to intake for: 3.2
                        // is another guestimate based off from how long the
                        // start auto takes,
                        AutoBuilder.followPath(path1)
                                .andThen(
                                        new CameraBallGatherCmd(r)
                                                .handleInterrupt(
                                                        (Runnable) new CloseBallGatherCmd(r))),
                        r.intake.smartIntake(),
                        r.shooter
                                .pointAtHub()
                                .withTimeout(endTime - prePrimeTime)
                                .andThen(
                                        new RunCommand(
                                                () ->
                                                        r.shooter.newPrime(
                                                                FieldConstants.Hub.center,
                                                                endPose1,
                                                                true))));
        sequence.addCommands(parallelGroup);
        sequence.addCommands(
                AutoBuilder.pathfindToPoseFlipped(endPose1, path2.getGlobalConstraints()));

        // shoot again for the remaining time
        sequence.addCommands(
                ShooterCommands.smartShoot(r, FieldConstants.Hub.center)
                        .alongWith(r.intake.shakeTheIntake())
                        .withTimeout(secondShootTime));

        return new ConditionalCommand(abortCommand(), sequence, this::shouldAbort);
    }

    public Command twoMiddleScoopAuto(PathPlannerPath path1, PathPlannerPath path2) {
        double initialShootWait = 1.0;
        double firstShootTime = 4;
        double secondShootTime = 5;

        SequentialCommandGroup sequence = new SequentialCommandGroup();
        // first drop the intake as fast as possible
        sequence.addCommands(
                ShooterCommands.smartShoot(r, FieldConstants.Hub.center)
                        .alongWith(new InstantCommand(r.intake::extend, r.intake))
                        .withTimeout(initialShootWait)
                        .finallyDo(
                                () -> {
                                    r.shooter.stopAll().execute();
                                    r.spindexter.stop().execute();
                                    r.intake.reallyExtend();
                                    r.intake.stopIntake().initialize();
                                }));

        Pose2d nextPathStart =
                new Pose2d(path2.getPoint(0).position, path2.getIdealStartingState().rotation());

        Pose2d endPose;
        double endTime;
        try {
            var end = path1.getIdealTrajectory(Drive.PP_CONFIG).get().getEndState();
            endPose = end.pose;
            endTime = end.timeSeconds;
        } catch (Exception e) {
            e.printStackTrace();
            endPose = nextPathStart;
            endTime = 5.5;
        }
        final Pose2d endPose1 = endPose;

        // drive the profile while intaking
        ParallelDeadlineGroup parallelGroup =
                new ParallelDeadlineGroup(
                        AutoBuilder.followPath(path1),
                        r.intake.smartIntake(),
                        r.shooter
                                .pointAtHub()
                                .withTimeout(endTime - prePrimeTime)
                                .andThen(
                                        new RunCommand(
                                                () ->
                                                        r.shooter.newPrime(
                                                                FieldConstants.Hub.center,
                                                                endPose1,
                                                                true))));
        sequence.addCommands(parallelGroup);

        // shoot the balls while potentially moving

        PathConstraints moveAndShootLimits = new PathConstraints(0.75, 0.75, 1, 1);
        sequence.addCommands(
                ShooterCommands.smartShoot(r, FieldConstants.Hub.center)
                        .alongWith(r.intake.shakeTheIntake())
                        .withTimeout(firstShootTime)
                        .finallyDo(
                                () -> {
                                    r.shooter.stopAll().execute();
                                    r.spindexter.stop().execute();
                                    r.intake.reallyExtend();
                                    r.intake.stopIntake().execute();
                                })
                        .alongWith(
                                AutoBuilder.pathfindToPoseFlipped(
                                        nextPathStart, moveAndShootLimits))
                        .andThen(
                                DriveCommands.driveToPoint(
                                        r,
                                        () -> FieldConstants.flipIfRed(nextPathStart),
                                        2,
                                        null)));
        // sequence.addCommands(r.intake.fastDrop());

        try {
            var end = path1.getIdealTrajectory(Drive.PP_CONFIG).get().getEndState();
            endPose = end.pose;
            endTime = end.timeSeconds;
        } catch (Exception e) {
            e.printStackTrace();
            endPose = nextPathStart;
            endTime = 5.5;
        }
        final Pose2d endPose2 = endPose;

        // drive the second profile while intaking
        parallelGroup =
                new ParallelDeadlineGroup(
                        AutoBuilder.followPath(path2),
                        r.intake.smartIntake(),
                        r.shooter
                                .pointAtHub()
                                .withTimeout(endTime - prePrimeTime)
                                .andThen(
                                        new RunCommand(
                                                () ->
                                                        r.shooter.newPrime(
                                                                FieldConstants.Hub.center,
                                                                endPose2,
                                                                true))));
        sequence.addCommands(parallelGroup);

        // shoot again for the remaining time
        sequence.addCommands(
                ShooterCommands.smartShoot(r, FieldConstants.Hub.center)
                        .withTimeout(secondShootTime));

        return new ConditionalCommand(abortCommand(), sequence, this::shouldAbort);
    }

    public Command singlePathPass(PathPlannerPath path) {
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addCommands(
                r.intake
                        .fastDrop()
                        .finallyDo(
                                () -> {
                                    r.shooter.stopAll().execute();
                                    r.spindexter.stop().execute();
                                    r.intake.reallyExtend();
                                    r.intake.stopIntake().initialize();
                                }));
        ParallelDeadlineGroup parallelGroup =
                new ParallelDeadlineGroup(
                        AutoBuilder.followPath(path),
                        r.intake.smartIntake(),
                        ShooterCommands.smartShoot(r, FieldConstants.Locations.passLeft));
        // passes pick the right direction regardless of the input now
        sequence.addCommands(parallelGroup);
        return sequence;
    }

    public Command singleScoopDepot(PathPlannerPath path) {
        double initialShootWait = 1.1;
        double firstShootTime = 5;
        double secondShootTime = 5;

        SequentialCommandGroup sequence = new SequentialCommandGroup();
        // first drop the intake as fast as possible
        sequence.addCommands(
                ShooterCommands.smartShoot(r, FieldConstants.Hub.center)
                        .alongWith(new InstantCommand(r.intake::extend, r.intake))
                        .withTimeout(initialShootWait)
                        .finallyDo(
                                () -> {
                                    r.shooter.stopAll().execute();
                                    r.spindexter.stop().execute();
                                    r.intake.reallyExtend();
                                    r.intake.stopIntake().initialize();
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
                                    r.intake.reallyExtend();
                                    r.intake.stopIntake().execute();
                                })
                        .alongWith(
                                AutoBuilder.pathfindToPoseFlipped(
                                        nextPathStart, moveAndShootLimits))
                        .andThen(
                                DriveCommands.driveToPoint(
                                        r,
                                        () -> FieldConstants.flipIfRed(nextPathStart),
                                        2,
                                        null)));
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
                                                                Rotation2d.k180deg)),
                                        2,
                                        null)
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

    private Command waitForTimeOrNoBalls(double time) {
        Debouncer debouncer = new Debouncer(0.6);
        return new WaitCommand(time)
                .raceWith(
                        new WaitCommand(1.0)
                                .andThen(
                                        new InstantCommand(() -> debouncer.calculate(false))
                                                .andThen(
                                                        new WaitUntilCommand(
                                                                () ->
                                                                        debouncer.calculate(
                                                                                r.spindexter
                                                                                                .noBallsInSpindexer()
                                                                                        && !r.shooter
                                                                                                .ballShotEdge)))));
    }

    private boolean isInNeutralZone() {
        Translation2d loc = r.drive.getPose().getTranslation();
        if (loc.getX() < FieldConstants.fieldLength - FieldConstants.Hub.center.getX()
                && loc.getX() > FieldConstants.Hub.center.getX()) {
            return true;
        } else {
            return false;
        }
    }

    public Command robotShake() {
        ShooterCommands.Thing<Rotation2d> rotationThing = new ShooterCommands.Thing<>();
        Command captureThing =
                new InstantCommand(() -> rotationThing.accept(r.drive.getRotation()));
        Rotation2d extra = Rotation2d.fromDegrees(5);

        SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addCommands(
                DriveCommands.joystickDriveAtAngle(
                                r.drive, () -> 0, () -> 0, () -> rotationThing.get().plus(extra))
                        .withTimeout(0.1));
        sequence.addCommands(
                DriveCommands.joystickDriveAtAngle(
                                r.drive, () -> 0, () -> 0, () -> rotationThing.get().minus(extra))
                        .withTimeout(0.1));

        return sequence.repeatedly().beforeStarting(captureThing);
    }
}
