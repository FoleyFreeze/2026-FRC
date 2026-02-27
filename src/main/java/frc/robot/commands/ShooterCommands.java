package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class ShooterCommands {

    // is this going to be a thing?
    public static class Thing<T> implements Supplier<T>, Consumer<T> {
        T thing;

        @Override
        public void accept(T value) {
            thing = value;
        }

        @Override
        public T get() {
            return thing;
        }
    }

    public static Command hubShoot(RobotContainer r) {
        return new RunCommand(
                        () -> r.shooter.newPrime(FieldConstants.Hub.center, r.drive.getPose()),
                        r.shooter)
                .alongWith(r.spindexter.smartSpinCmd(r.shooter, r.drive));
    }

    public static Command smartHubShoot(RobotContainer r, CommandXboxController controller) {
        Thing<Rotation2d> rotationThing = new Thing<>(); // thing1
        Thing<Double> velocityThing = new Thing<>(); // thing2

        return new RunCommand(
                        () ->
                                r.shooter.newPrime(
                                        FieldConstants.Hub.center,
                                        r.drive.getPose(),
                                        rotationThing,
                                        velocityThing),
                        r.shooter)
                .alongWith(
                        DriveCommands.driveAtAngleFFw(
                                r.drive,
                                () -> -controller.getLeftY(),
                                () -> -controller.getLeftX(),
                                rotationThing,
                                velocityThing))
                .alongWith(r.spindexter.smartSpinCmd(r.shooter, r.drive));
    }
}
