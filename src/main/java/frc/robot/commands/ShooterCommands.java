package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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

    public static Command smarterShoot(
            RobotContainer r, CommandXboxController controller, Translation2d target) {
        Thing<Rotation2d> rotationThing = new Thing<>(); // thing1
        Thing<Double> velocityThing = new Thing<>(); // thing2

        // complex use of debounce, but we are looking for if time has elapsed since shooting a ball
        Debouncer shotDebounce = new Debouncer(0.75, DebounceType.kFalling);

        // index sequence
        SequentialCommandGroup indexerSequence = new SequentialCommandGroup();
        // first reset debouncer as if we have just made a shot
        indexerSequence.addCommands(new InstantCommand(() -> shotDebounce.calculate(true)));
        // then run indexer until it gets jammed
        indexerSequence.addCommands(
                r.spindexter
                        .smartSpinCmd(r.shooter, r.drive)
                        .until(() -> shotDebounce.calculate(r.shooter.ballShotEdge)));
        // then run the unjam sequence
        indexerSequence.addCommands(r.spindexter.smartUnjam().withDeadline(new WaitCommand(0.5)));

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
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        rotationThing,
                        velocityThing));
        parallelGroup.addCommands(indexerSequence.repeatedly());

        return parallelGroup;
    }
}
