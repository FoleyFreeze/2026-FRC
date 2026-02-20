package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ClimbCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.shooter.Shooter.ManualShotLoc;

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

        // add drive over bump
        // add drive through trench

        // zero drive
        // TODO: blink LEDs or something
        controller.start().debounce(2).onTrue(new InstantCommand(() -> r.drive.zeroDrive()));

        // intake functions
        // TODO: Victor
        // intake in M2
        // intake out M6

        controller.a().onTrue(new InstantCommand(r.intake::extend));
        controller.b().onTrue(new InstantCommand(r.intake::retract));

        // camera gather M5
        // unjam A

        // shoot functions
        // pass left LB

        controller
                .leftBumper()
                .whileTrue(
                        new RunCommand(
                                        () ->
                                                r.shooter.newPrime(
                                                        FieldConstants.Locations.passLeft,
                                                        r.drive.getPose()),
                                        r.shooter)
                                .alongWith(r.spindexter.smartSpinCmd(r.shooter, r.drive)));
        // // pass right RB
        controller
                .rightBumper()
                .whileTrue(
                        new RunCommand(
                                        () ->
                                                r.shooter.newPrime(
                                                        FieldConstants.Locations.passRight,
                                                        r.drive.getPose()),
                                        r.shooter)
                                .alongWith(r.spindexter.smartSpinCmd(r.shooter, r.drive)));

        // shoot hub RT
        controller
                .rightTrigger()
                .whileTrue(
                        new RunCommand(
                                        () ->
                                                r.shooter.newPrime(
                                                        FieldConstants.Hub.center,
                                                        r.drive.getPose()),
                                        r.shooter)
                                .alongWith(r.spindexter.smartSpinCmd(r.shooter, r.drive)));
        // manual shoot LT
        controller
                .leftTrigger()
                .whileTrue(
                        r.shooter
                                .manualPrimeCmd()
                                .alongWith(r.spindexter.smartSpinCmd(r.shooter, r.drive)));
        // set manual shot positions (X Y B)

        controller
                .x()
                .onTrue(new InstantCommand(() -> r.shooter.setManualGoal(ManualShotLoc.CLIMB)));
        controller
                .y()
                .onTrue(new InstantCommand(() -> r.shooter.setManualGoal(ManualShotLoc.FRONT_HUB)));

        // select zero turret (reset to abs enc)
        controller.button(7).onTrue(new InstantCommand()); // TODO: add the part that does stuff
        // start+select full zero turret (reset to zero and ignore abs)

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
