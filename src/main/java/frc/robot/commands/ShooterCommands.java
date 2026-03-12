package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

    public static Command smartShoot(
            RobotContainer r, CommandXboxController controller, Translation2d target) {
        Thing<Rotation2d> rotationThing = new Thing<>(); // thing1
        Thing<Double> velocityThing = new Thing<>(); // thing2

        return new RunCommand(
                        () ->
                                r.shooter.newPrime(
                                        target, r.drive.getPose(), rotationThing, velocityThing),
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

    public static Command smarterShootNoGather(
            RobotContainer r, CommandXboxController controller, Translation2d target) {
        Thing<Rotation2d> rotationThing = new Thing<>(); // thing1
        Thing<Double> velocityThing = new Thing<>(); // thing2

        SequentialCommandGroup shakeTheIntake = new SequentialCommandGroup();
        shakeTheIntake.addCommands(new InstantCommand(() -> r.intake.extend()));
        shakeTheIntake.addCommands(r.intake.stopIntake());
        shakeTheIntake.addCommands(new WaitCommand(1.5));
        shakeTheIntake.addCommands(new InstantCommand(() -> r.intake.retract()));
        shakeTheIntake.addCommands(r.intake.smartIntake().withTimeout(0.75));

        // run shoot, drive, and index in parallel
        ParallelCommandGroup parallelGroup = new ParallelCommandGroup();
        parallelGroup.addCommands(
                new RunCommand(
                        () ->
                                r.shooter.newPrime(
                                        target, r.drive.getPose(), rotationThing, velocityThing),
                        r.shooter));
        // only drive if a controller was provided
        if (controller != null) {
            parallelGroup.addCommands(
                    DriveCommands.driveAtAngleFFw(
                            r.drive,
                            () -> -controller.getLeftY() * 0.87, // 0.87 is 0.7^2.5
                            () -> -controller.getLeftX() * 0.87,
                            rotationThing,
                            velocityThing));
        }
        parallelGroup.addCommands(r.spindexter.smarterSpinCmd());
        parallelGroup.addCommands(shakeTheIntake.repeatedly());

        return parallelGroup;
    }

    public static Command smarterShootAndGather(
            RobotContainer r, CommandXboxController controller, Translation2d target) {
        Thing<Rotation2d> rotationThing = new Thing<>(); // thing1
        Thing<Double> velocityThing = new Thing<>(); // thing2

        // run shoot, drive, and index in parallel
        ParallelCommandGroup parallelGroup = new ParallelCommandGroup();
        parallelGroup.addCommands(
                new RunCommand(
                        () ->
                                r.shooter.newPrime(
                                        target, r.drive.getPose(), rotationThing, velocityThing),
                        r.shooter));
        parallelGroup.addCommands(
                DriveCommands.driveAtAngleFFw(
                        r.drive,
                        () -> -controller.getLeftY() * 0.87, // 0.87 is 0.7^2.5
                        () -> -controller.getLeftX() * 0.87,
                        rotationThing,
                        velocityThing));
        parallelGroup.addCommands(r.spindexter.smarterSpinCmd());
        parallelGroup.addCommands(r.intake.smartIntake());

        return parallelGroup;
    }
}
