package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;

public class ConfigButtons {

    public static enum manualShotLoc {
        CLIMB, // x
        FRONT_HUB, // y
        RIGHT_BACK_CORNER // b
    }

    private static manualShotLoc manualShotLocState;

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

        // add drive over bump
        // add drive through trench

        // zero drive
        // TODO: blink LEDs or something
        controller.start().debounce(2).onTrue(new InstantCommand(() -> r.drive.zeroDrive()));

        // intake functions
        // TODO: Victor
        // intake in M2
        // intake out M6
        // camera gather M5
        // unjam A

        // shoot functions
        // pass left LB
        controller
                .leftBumper()
                .onTrue(new InstantCommand(() -> r.shooter.newPrime(FieldConstants.passLeft, r)));
        // pass right RB
        controller
                .rightBumper()
                .onTrue(new InstantCommand(() -> r.shooter.newPrime(FieldConstants.passRight, r)));
        // shoot hub RT 
        //TODO: BELOW
            // controller
            //         .rightTrigger()
            //         .whileTrue(
            //                 new InstantCommand(() -> r.shooter.newPrime(FieldConstants.Hub.center, r))).whileTrue(new InstantCommand(()-> r.spindexter.spin()));

            // controller.rightTrigger().whileFalse(new InstantCommand(()-> r.shooter.stop()));
        //TODO: ABOVE
        // manual shoot LT

        // set manual shot positions (X Y B)
        controller.x().onTrue(new InstantCommand(() -> manualShotLocState = manualShotLoc.CLIMB));
        controller
                .y()
                .onTrue(new InstantCommand(() -> manualShotLocState = manualShotLoc.FRONT_HUB));
        controller
                .b()
                .onTrue(
                        new InstantCommand(
                                () -> manualShotLocState = manualShotLoc.RIGHT_BACK_CORNER));
        // select zero turret (reset to abs enc)
        controller.button(7).onTrue(new InstantCommand()); // TODO: add the part that does stuff
        // start+select full zero turret (reset to zero and ignore abs)

        // climb functions
        // dpad + X drive then climb
        // dpad just climb

        // operator board
        // mode sw (idk what we want this to do yet)
        // jogs (shoot speed, angle, etc)

    }
}
