package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;

public class ConfigButtons {

    // Controller
    private static final CommandXboxController controller = new CommandXboxController(0);

    public static void config(RobotContainer r) {

        //drive functions
        r.drive.setDefaultCommand(
            DriveCommands.joystickDrive(r.drive, 
                () -> -controller.getLeftY(), 
                () -> -controller.getLeftX(), 
                () -> -controller.getRightX()
        ));

        //add drive over bump
        //add drive through trench

        //zero drive
        //TODO: blink LEDs or something
        controller.start().debounce(2).onTrue(new InstantCommand(() -> r.drive.zeroDrive()));


        //intake functions
        //TODO: Victor
        //intake in M2
        //intake out M6
        //camera gather M5
        //unjam A


        //shoot functions
        //pass left LB
        //pass right RB
        //shoot hub RT
        //manual shoot LT
        //set manual shot positions (X Y B)

        //select zero turret (reset to abs enc)
        //start+select full zero turret (reset to zero and ignore abs)


        //climb functions
        //dpad + X drive then climb
        //dpad just climb


        //operator board
        //mode sw (idk what we want this to do yet)
        //jogs (shoot speed, angle, etc)

    }
}
