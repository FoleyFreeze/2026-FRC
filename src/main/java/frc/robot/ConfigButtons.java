package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.shooter.Shooter.ManualShotLoc;

public class ConfigButtons {

    // Controller
    private static final CommandXboxController controller = new CommandXboxController(0);
    public static final CommandJoystick driveStation = new CommandJoystick(3);
    private static final CommandJoystick driveStation2 = new CommandJoystick(4);
    // note that xbox controller takes the first 3 spots

    public static void config(RobotContainer r) {

        // drive functions
        final double thetaReduction = Math.pow(0.65, 1.0 / DriveCommands.thetaExpo);
        r.drive.setDefaultCommand(
                DriveCommands.joystickDriveTurnOut(
                        r.drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> -controller.getRightX() * thetaReduction));
        r.shooter.setDefaultCommand(r.shooter.stop());
        r.spindexter.setDefaultCommand(r.spindexter.stop());
        r.intake.setDefaultCommand(r.intake.stopIntake());

        // drive over trench
        // 0.65 ^ 2.5 is 0.35
        // 0.69 ^ 2.5 is 0.40
        // 0.76 ^ 2.5 is 0.50
        // 0.82 ^ 2.5 is 0.60
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
        // TODO: Victor
        // intake in M2
        // intake out M6

        controller.a().onTrue(new InstantCommand(r.intake::extend));

        controller.b().onTrue(new InstantCommand(r.intake::retract));

        // intake spin
        controller
                .leftTrigger()
                .and(controller.rightTrigger().negate())
                .whileTrue(r.intake.smartIntake());

        // camera gather M5
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
                .whileTrue(
                        ShooterCommands.smartShoot(
                                r, controller, FieldConstants.Locations.passLeft));
        // // pass right RB
        controller
                .rightBumper()
                .whileTrue(
                        ShooterCommands.smartShoot(
                                r, controller, FieldConstants.Locations.passRight));

        // shoot hub RT
        final double shootXyReduce = Math.pow(0.7, 1.0 / DriveCommands.xyExpo);
        controller
                .rightTrigger()
                .and(controller.leftTrigger().negate())
                .whileTrue(
                        ShooterCommands.smarterShootNoGather(
                                r,
                                () -> -controller.getLeftY() * shootXyReduce,
                                () -> -controller.getLeftX() * shootXyReduce,
                                FieldConstants.Hub.center));

        controller
                .rightTrigger()
                .and(controller.leftTrigger())
                .whileTrue(
                        ShooterCommands.smarterShootAndGather(
                                r,
                                () -> -controller.getLeftY(),
                                () -> -controller.getLeftX(),
                                FieldConstants.Hub.center));

        // set manual shot positions (X Y B)

        // controller
        //         .x()
        //         .and(controller.leftTrigger())
        //         .whileTrue(
        //                 new InstantCommand(() -> r.shooter.setManualGoal(ManualShotLoc.CLIMB))
        //                         .andThen(
        //                                 r.shooter
        //                                         .manualShot()
        //                                         .alongWith(
        //                                                 r.spindexter.smartSpinCmd(
        //                                                         r.shooter, r.drive))));
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
        // controller
        //         .y()
        //         .and(controller.leftTrigger())
        //         .whileTrue(
        //                 new InstantCommand(() ->
        // r.shooter.setManualGoal(ManualShotLoc.FRONT_HUB))
        //                         .andThen(
        //                                 r.shooter
        //                                         .manualShot()
        //                                         .alongWith(r.spindexter.smarterSpinCmd())));
        controller
                .povUp()
                .whileTrue(
                        new InstantCommand(() -> r.shooter.setManualGoal(ManualShotLoc.FRONT_HUB))
                                .andThen(
                                        r.shooter
                                                .manualShot()
                                                .alongWith(r.spindexter.smarterSpinCmd())));

        // select zero turret (reset to abs enc)
        // start+select full zero turret (reset to zero and ignore abs)

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
    }

    public static double trackButtons() {
        double count = 0;

        int buttoncount = controller.getHID().getButtonCount();
        for (int i = 1; i < buttoncount + 1; i++) {
            if (controller.getHID().getRawButtonPressed(i)) {
                count++;
            }
        }

        return count;
    }
}
