package frc.robot;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.shooter.Shooter.ManualShotLoc;
import frc.robot.util.EdgeDetector;
import frc.robot.util.EdgeDetector.EdgeType;
import java.util.ArrayList;

public class ConfigButtons {

    // Controller
    private static final CommandXboxController controller = new CommandXboxController(0);
    public static final CommandJoystick driveStation = new CommandJoystick(3);
    private static final CommandJoystick driveStation2 = new CommandJoystick(4);
    // note that xbox controller takes the first 3 spots

    public static void config(RobotContainer r) {
        Trigger botDisabled = new Trigger(() -> DriverStation.isDisabled());

        // drive functions
        final double thetaReduction = Math.pow(0.58, 1.0 / DriveCommands.thetaExpo);
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
                .or(controller.leftBumper())
                .whileTrue(
                        DriveCommands.joystickDriveTurnOut(
                                r.drive,
                                () -> -controller.getLeftY() * shootXyReduce,
                                () -> -controller.getLeftX() * shootXyReduce,
                                () -> -controller.getRightX() * shootZReduce));

        // drive over trench
        final double xyReduceBump = Math.pow(0.5, 1.0 / DriveCommands.xyExpo);
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
        controller.leftTrigger().whileTrue(r.intake.smartIntake());

        // shake while shoot unless gathering
        controller
                .rightTrigger()
                .or(controller.rightBumper())
                .or(controller.leftBumper())
                .and(controller.leftTrigger().negate())
                .whileTrue(r.intake.shakeTheIntake());

        // camera gather (LB maybe)

        // unjam back
        controller
                .x()
                .whileTrue(
                        new RunCommand(() -> r.spindexter.unjam(), r.spindexter)
                                .alongWith(r.intake.unjamIntake()));

        // shoot functions
        // pass left LB
        controller
                .leftBumper()
                .whileTrue(ShooterCommands.smartShoot(r, FieldConstants.Locations.passLeft));
        // // pass right RB
        controller
                .rightBumper()
                .whileTrue(ShooterCommands.smartShoot(r, FieldConstants.Locations.passRight));

        // shoot hub RT
        controller
                .rightTrigger()
                .whileTrue(ShooterCommands.smartShoot(r, FieldConstants.Hub.center));

        // when not shooting and in autoPoint mode, point at the hub
        // shooter cam switch is active low
        driveStation
                .button(10)
                .negate()
                .and(
                        controller
                                .rightTrigger()
                                .or(controller.rightBumper())
                                .or(controller.leftBumper())
                                .negate())
                .and(botDisabled.negate())
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

        // select zero turret? (reset to abs enc)
        // start+select full zero turret? (reset to zero and ignore abs)

        // climb functions
        // dpad + X drive then climb
        // controller
        //         .x()
        //         .and(controller.povLeft())
        //         .whileTrue(
        //                 ClimbCommands.autoClimb(
        //                         r,
        //                         () ->
        //                                 FieldConstants.flipIfRed(
        //                                         FieldConstants.Locations.towerLeftFrontPose2d),
        //                         Units.inchesToMeters(12)));
        // controller
        //         .x()
        //         .and(controller.povRight())
        //         .whileTrue(
        //                 ClimbCommands.autoClimb(
        //                         r,
        //                         () ->
        //                                 FieldConstants.flipIfRed(
        //                                         FieldConstants.Locations.towerRightFrontPose2d),
        //                         Units.inchesToMeters(12)));
        // dpad just climb
        // controller.povUp().whileTrue(r.climber.autoExtendCmd());

        // dpad down to declimb
        // controller.povDown().onTrue(ClimbCommands.autoDown(r));

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
