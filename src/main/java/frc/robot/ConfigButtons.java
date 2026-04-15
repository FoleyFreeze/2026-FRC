package frc.robot;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CameraBallGatherCmd;
import frc.robot.commands.CloseBallGatherCmd;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.ShooterCommands.Thing;
import frc.robot.subsystems.shooter.Shooter.ManualShotLoc;
import frc.robot.util.EdgeDetector;
import frc.robot.util.EdgeDetector.EdgeType;
import java.util.ArrayList;

public class ConfigButtons {

    // Controller
    public static final CommandXboxController controller = new CommandXboxController(0);
    public static final CommandJoystick driveStation = new CommandJoystick(3);
    private static final CommandJoystick driveStation2 = new CommandJoystick(4);
    // note that xbox controller takes the first 3 spots

    // used to enable/disable shooting in opponents shift
    public static Trigger fieldOrientSw = driveStation.button(7);
    // used as turret enable switch
    public static Trigger shotCamSw = driveStation.button(10).negate();
    // used to enable/disable path generation for ball gather vs just driving to closest ball
    public static Trigger ballCamSw = driveStation.button(6);

    public static void config(RobotContainer r) {
        Trigger botDisabled = new Trigger(() -> DriverStation.isDisabled());

        // drive functions
        final double thetaReduction = Math.pow(0.50, 1.0 / DriveCommands.thetaExpo);
        r.drive.setDefaultCommand(
                DriveCommands.joystickDriveTurnOut(
                        r.drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> -controller.getRightX() * thetaReduction));
        r.shooter.setDefaultCommand(r.shooter.stopAll());
        r.spindexter.setDefaultCommand(r.spindexter.stop());
        r.intake.setDefaultCommand(r.intake.stopIntake());

        // drive while shooting
        final double shootXyReduce = Math.pow(0.7, 1.0 / DriveCommands.xyExpo);
        final double shootZReduce = Math.pow(0.45, 1.0 / DriveCommands.thetaExpo);
        controller
                .rightTrigger()
                .or(controller.rightBumper())
                // dont run if turret disabled
                .and(shotCamSw)
                // let autogather keep control of the drivetrain
                .and(controller.leftBumper().negate())
                .whileTrue(
                        DriveCommands.joystickDriveTurnOut(
                                r.drive,
                                () -> -controller.getLeftY() * shootXyReduce,
                                () -> -controller.getLeftX() * shootXyReduce,
                                () -> -controller.getRightX() * shootZReduce));

        Thing<Rotation2d> rotationThing = new Thing<>();
        Thing<Double> velocityThing = new Thing<>();
        controller
                .rightTrigger()
                .or(controller.rightBumper())
                // run if turret disabled
                .and(shotCamSw.negate())
                .whileTrue(
                        DriveCommands.driveAtAngleFFw(
                                r.drive,
                                () -> -controller.getLeftY() * shootXyReduce,
                                () -> -controller.getLeftX() * shootXyReduce,
                                rotationThing,
                                velocityThing));

        // drive over trench
        final double xyReduceBump = Math.pow(0.6, 1.0 / DriveCommands.xyExpo);
        final double xyReduceTrench = Math.pow(0.7, 1.0 / DriveCommands.xyExpo);
        controller
                .leftStick()
                .whileTrue(
                        DriveCommands.joystickDriveAtAngle(
                                r.drive,
                                () -> -controller.getLeftY() * xyReduceTrench,
                                () -> -controller.getLeftX() * xyReduceTrench,
                                r.drive::getTrenchAngle));
        // add drive through bump
        controller
                .rightStick()
                .whileTrue(
                        DriveCommands.joystickDriveAtAngle(
                                r.drive,
                                () -> -controller.getLeftY() * xyReduceBump,
                                () -> -controller.getLeftX() * xyReduceBump,
                                r.drive::getBumpAngle));

        // zero drive
        // TODO: blink LEDs or something
        controller.start().debounce(0.5).onTrue(new InstantCommand(() -> r.drive.zeroDrive()));

        // intake functions
        // intake in
        // intake out
        controller.a().onTrue(new InstantCommand(r.intake::extend));
        controller.b().onTrue(new InstantCommand(r.intake::retract));

        // intake spin
        controller
                .leftTrigger()
                .or(controller.leftBumper())
                .whileTrue(
                        r.intake
                                .smartIntake()
                                .beforeStarting(new InstantCommand(r.intake::reallyExtend))
                                .finallyDo(r.intake::extend));

        // shake while shoot unless gathering
        controller
                .rightTrigger()
                .or(controller.rightBumper())
                .and(controller.leftTrigger().negate().and(controller.leftBumper().negate()))
                .whileTrue(r.intake.shakeTheIntake());

        // camera gather (LB maybe)
        controller.leftBumper().and(ballCamSw).whileTrue(new CameraBallGatherCmd(r));
        controller
                .leftBumper()
                .and(ballCamSw.negate())
                .whileTrue(new CloseBallGatherCmd(r).repeatedly());

        // unjam back
        controller
                .x()
                .whileTrue(
                        new RunCommand(() -> r.spindexter.unjam(), r.spindexter)
                                .alongWith(r.intake.unjamIntake()));

        // shoot functions
        // pass left LB (has been stolen for auto gather)
        // controller
        //         .leftBumper()
        //         .and(driveStation.button(10).negate()) // with turret
        //         .whileTrue(ShooterCommands.smartShoot(r, FieldConstants.Locations.passLeft));
        // controller
        //         .leftBumper()
        //         .and(driveStation.button(10)) // without turret
        //         .whileTrue(
        //                 ShooterCommands.smartShootNoTurret(
        //                         r,
        //                         FieldConstants.Locations.passLeft,
        //                         rotationThing,
        //                         velocityThing));
        // pass right RB
        controller
                .rightBumper()
                .and(shotCamSw) // with turret
                .whileTrue(ShooterCommands.smartShoot(r, FieldConstants.Locations.passRight));
        controller
                .rightBumper()
                .and(shotCamSw.negate()) // without turret
                .whileTrue(
                        ShooterCommands.smartShootNoTurret(
                                r,
                                FieldConstants.Locations.passRight,
                                rotationThing,
                                velocityThing));

        // shoot hub RT
        controller
                .rightTrigger()
                .and(shotCamSw)
                .whileTrue(ShooterCommands.smartShoot(r, FieldConstants.Hub.center));
        controller
                .rightTrigger()
                .and(shotCamSw.negate())
                .whileTrue(
                        ShooterCommands.smartShootNoTurret(
                                r, FieldConstants.Hub.center, rotationThing, velocityThing));

        // when not shooting and in autoPoint mode, point at the hub
        // shooter cam switch is active low
        driveStation
                .button(10)
                .negate()
                .and(controller.rightTrigger().or(controller.rightBumper()).negate())
                .and(botDisabled.negate())
                .and(new Trigger(() -> !DriverStation.isAutonomous()))
                .whileTrue(r.shooter.pointAtHub());

        // set manual shot positions (dpad)
        controller
                .povDown()
                .whileTrue(
                        new InstantCommand(() -> r.shooter.setManualGoal(ManualShotLoc.CLIMB))
                                .andThen(
                                        r.shooter
                                                .manualShot()
                                                .alongWith(
                                                        r.spindexter.smartSpinCmd(
                                                                r.shooter, r.drive))));

        controller
                .povUp()
                .whileTrue(
                        new InstantCommand(() -> r.shooter.setManualGoal(ManualShotLoc.FRONT_HUB))
                                .andThen(
                                        r.shooter
                                                .manualShot()
                                                .alongWith(r.spindexter.smarterSpinCmd())));

        // operator board
        // mode sw (idk what we want this to do yet)
        // jogs (shoot speed, angle, etc)

        driveStation
                .axisLessThan(1, -0.5)
                .and(driveStation2.button(12))
                .onTrue(r.shooter.jogPassRpmUp());
        driveStation
                .axisLessThan(1, -0.5)
                .and(driveStation2.button(12).negate())
                .onTrue(r.shooter.jogHubRpmUp());
        driveStation
                .axisGreaterThan(1, 0.5)
                .and(driveStation2.button(12))
                .onTrue(r.shooter.jogPassRpmDn());
        driveStation
                .axisGreaterThan(1, 0.5)
                .and(driveStation2.button(12).negate())
                .onTrue(r.shooter.jogHubRpmDn());

        driveStation
                .axisLessThan(0, -0.5)
                .and(driveStation2.button(12))
                .onTrue(r.shooter.jogPassAngleDn());
        driveStation
                .axisLessThan(0, -0.5)
                .and(driveStation2.button(12).negate())
                .onTrue(r.shooter.jogHubAngleDn());
        driveStation
                .axisGreaterThan(0, 0.5)
                .and(driveStation2.button(12))
                .onTrue(r.shooter.jogPassAngleUp());
        driveStation
                .axisGreaterThan(0, 0.5)
                .and(driveStation2.button(12).negate())
                .onTrue(r.shooter.jogHubAngleUp());

        // other commands
        botDisabled.debounce(5, DebounceType.kRising).whileTrue(r.shooter.recalcTurretToEnc());

        // zero turret to abs enc
        driveStation
                .button(11)
                .negate()
                .and(driveStation2.button(12))
                .debounce(0.5, DebounceType.kRising)
                .onTrue(new InstantCommand(() -> r.shooter.zeroTurret()).ignoringDisable(true));
    }

    static ArrayList<EdgeDetector> controllerEdges = new ArrayList<>();
    static ArrayList<EdgeDetector> dsEdges = new ArrayList<>();
    static ArrayList<EdgeDetector> ds2Edges = new ArrayList<>();

    static {
        // initialize all edge detectors
        for (int i = 0; i < 2; i++) {
            controllerEdges.add(new EdgeDetector(EdgeType.RISING));
        }
        for (int i = 0; i < 4; i++) {
            dsEdges.add(new EdgeDetector(EdgeType.RISING));
        }
    }

    public static double trackButtons() {
        double count = 0;

        int buttoncount = controller.getHID().getButtonCount();
        for (int i = 1; i < buttoncount + 1; i++) {
            if (controller.getHID().getRawButtonPressed(i)) {
                count++;
            }
        }

        // count axis as buttons (custom edge detector)
        if (controllerEdges.get(0).calc(controller.getHID().getLeftTriggerAxis() > 0.5)) {
            count++;
        }
        if (controllerEdges.get(1).calc(controller.getHID().getRightTriggerAxis() > 0.5)) {
            count++;
        }
        return count;
    }

    public static double trackControlBoardButtons() {
        double count = 0;

        int buttoncount = driveStation.getHID().getButtonCount();
        for (int i = 1; i < buttoncount + 1; i++) {
            if (driveStation.getHID().getRawButtonPressed(i)) {
                count++;
            }
        }
        // count axis as buttons
        if (dsEdges.get(0).calc(driveStation.getHID().getRawAxis(0) > 0.5)) {
            count++;
        }
        if (dsEdges.get(1).calc(driveStation.getHID().getRawAxis(0) < -0.5)) {
            count++;
        }
        if (dsEdges.get(2).calc(driveStation.getHID().getRawAxis(1) > 0.5)) {
            count++;
        }
        if (dsEdges.get(3).calc(driveStation.getHID().getRawAxis(1) < -0.5)) {
            count++;
        }

        buttoncount = driveStation2.getHID().getButtonCount();
        for (int i = 1; i < buttoncount + 1; i++) {
            if (driveStation2.getHID().getRawButtonPressed(i)) {
                count++;
            }
        }
        return count;
    }
}
