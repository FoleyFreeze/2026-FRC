package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ClimbCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.shooter.Shooter.ManualShotLoc;
import frc.robot.subsystems.spindexter.Spindexter;

public class ConfigButtons {

    // Controller
    private static final CommandXboxController controller = new CommandXboxController(0);

    public static void config(RobotContainer r) {

        // drive functions
        r.drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        r.drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> -controller.getRightX()));
        r.shooter.setDefaultCommand(r.shooter.stop());
        r.spindexter.setDefaultCommand(r.spindexter.stop());

        // add drive over trench
        controller
                .leftStick()
                .whileTrue(
                        DriveCommands.joystickDriveAtAngle(
                                r.drive,
                                () -> -controller.getLeftY(),
                                () -> -controller.getLeftX(),
                                r.drive::getTrenchAngle));
        // add drive through bump
        controller
                .rightStick()
                .whileTrue(
                        DriveCommands.joystickDriveAtAngle(
                                r.drive,
                                () -> -controller.getLeftY(),
                                () -> -controller.getLeftX(),
                                r.drive::getBumpAngle));

        // zero drive
        // TODO: blink LEDs or something
        controller.start().debounce(2).onTrue(new InstantCommand(() -> r.drive.zeroDrive()));

        // intake functions
        // TODO: Victor
        // intake in M2
        // intake out M6

        controller.a().onTrue(new InstantCommand(r.intake::extend));
        controller.b().debounce(0.2).onTrue(new InstantCommand(r.intake::retract)); // require holding button but only for a tiny bit

        // camera gather M5
        // unjam back
        controller.back().whileTrue(new RunCommand(()-> r.spindexter.unjam(), r.spindexter));

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
        controller
                .rightTrigger()
                .whileTrue(ShooterCommands.smartShoot(r, controller, FieldConstants.Hub.center));

        // set manual shot positions (X Y B)

        controller
                .x().and(controller.leftTrigger())
                .whileTrue(new InstantCommand(() -> r.shooter.setManualGoal(ManualShotLoc.CLIMB)).andThen(r.shooter
                                .manualPrimeCmd()
                                .alongWith(r.spindexter.smartSpinCmd(r.shooter, r.drive))));
        controller
                .y().and(controller.leftTrigger())
                .whileTrue(new InstantCommand(() -> r.shooter.setManualGoal(ManualShotLoc.FRONT_HUB)).andThen(r.shooter
                                .manualPrimeCmd()
                                .alongWith(r.spindexter.smartSpinCmd(r.shooter, r.drive))));


        // select zero turret (reset to abs enc)
        // start+select full zero turret (reset to zero and ignore abs)

        // climb functions
        // dpad + X drive then climb
        controller
                .x()
                .and(controller.povLeft())
                .whileTrue(
                        ClimbCommands.autoClimb(
                                r,
                                () ->
                                        FieldConstants.flipIfRed(
                                                FieldConstants.Locations.towerLeftFrontPose2d),
                                Units.inchesToMeters(12)));
        controller
                .x()
                .and(controller.povRight())
                .whileTrue(
                        ClimbCommands.autoClimb(
                                r,
                                () ->
                                        FieldConstants.flipIfRed(
                                                FieldConstants.Locations.towerRightFrontPose2d),
                                Units.inchesToMeters(12)));
        // dpad just climb
        controller.povUp().whileTrue(r.climber.autoExtendCmd());

        // dpad down to declimb
        controller.povDown().onTrue(ClimbCommands.autoDown(r));

        // operator board
        // mode sw (idk what we want this to do yet)
        // jogs (shoot speed, angle, etc)

    }
}
