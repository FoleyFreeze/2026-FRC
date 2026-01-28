package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;

public class ConfigButtons {

    public enum ControllerState {
        INIT,
        NORMAL,
        CLIMB,
        MANUAL_SHOOT,
        MANUAL_UNJAM,
        MANUAL_CLIMB,
    }

    public static ControllerState conState = ControllerState.INIT;

    private static Trigger isInit = new Trigger(() -> conState == ControllerState.INIT);
    private static Trigger isNormal = new Trigger(() -> conState == ControllerState.NORMAL);
    private static Trigger isClimb = new Trigger(() -> conState == ControllerState.CLIMB);
    private static Trigger isManualShoot =
            new Trigger(() -> conState == ControllerState.MANUAL_SHOOT);
    private static Trigger isManualUnjam =
            new Trigger(() -> conState == ControllerState.MANUAL_UNJAM);
    private static Trigger isManualClimb =
            new Trigger(() -> conState == ControllerState.MANUAL_CLIMB);
    private static Trigger isEnabled = new Trigger(() -> DriverStation.isEnabled());

    // Controller
    private static final CommandXboxController controller = new CommandXboxController(0);

    public static void config(RobotContainer r) {
        registerStateTransitions();

        // as soon as the scheduler runs switch out of init mode and into normal mode
        CommandScheduler.getInstance()
                .schedule(
                        new WaitCommand(0.1)
                                .andThen(
                                        new InstantCommand(() -> conState = ControllerState.NORMAL))
                                .ignoringDisable(true));

        r.drive.setDefaultCommand(DriveCommands.joystickDrive(r.drive, () -> 0, () -> 0, () -> 0));

        isNormal.and(controller.rightTrigger().negate())
                .whileTrue(
                        DriveCommands.joystickDrive(
                                        r.drive,
                                        () -> -controller.getLeftY(),
                                        () -> -controller.getLeftX(),
                                        () -> -controller.getRightX())
                                .ignoringDisable(true));

        isClimb.whileTrue(
                DriveCommands.joystickDrive(
                                r.drive,
                                () -> -controller.getLeftY() * 0.5,
                                () -> -controller.getLeftX() * 0.5,
                                () -> -controller.getRightX() * 0.5)
                        .ignoringDisable(true));

        // Reset gyro / odometry
        final Runnable resetGyro =
                () ->
                        r.drive.setPose(
                                new Pose2d(
                                        r.drive.getPose().getTranslation(),
                                        new Rotation2d())); // zero gyro

        // controller.start().onTrue(Commands.runOnce(resetGyro, r.drive).ignoringDisable(true));

        // intake
        controller.a().whileTrue(r.intake.intakeDown());
        controller.b().whileTrue(r.intake.intakeUp());

        // shooter wheel
        // controller.leftTrigger().whileTrue(r.shooter.prime());
        isNormal.and(controller
                .leftTrigger())
                .whileTrue(r.shooter.cameraShoot(r.drive::getPose, r.drive::getFieldVelocity));
        controller.leftTrigger().whileFalse(r.shooter.stop());
        // spindexer
        isNormal.and(controller
                .rightTrigger())
                .whileTrue(
                        r.spindexter
                                .smartSpinCmd(r.shooter, r.drive)
                                .alongWith(
                                        DriveCommands.slowDrive(
                                                controller, r.drive))); // , botLoc, hubPos
        controller.rightTrigger().whileFalse(r.spindexter.stop());


        //TODO: manual shoot cmds
        isManualShoot.and(controller.leftTrigger()).whileTrue(r.shooter.prime());
        isManualShoot.and(controller.rightTrigger()).whileTrue(r.spindexter.spin());

    }

    // cycle between normal -> climb -> manual shoot -> manual unjam -> manual climb -> normal
    private static void registerStateTransitions() {
        controller
                .back()
                .and(controller.start())
                .and(isNormal)
                .onFalse(
                        new InstantCommand(() -> conState = ControllerState.CLIMB)
                                .ignoringDisable(true));

        controller
                .back()
                .and(controller.start())
                .and(isClimb)
                .and(isEnabled)
                .onFalse(
                        new InstantCommand(() -> conState = ControllerState.NORMAL)
                                .ignoringDisable(true));
        controller
                .back()
                .and(controller.start())
                .and(isClimb)
                .and(isEnabled.negate())
                .onFalse(
                        new InstantCommand(() -> conState = ControllerState.MANUAL_SHOOT)
                                .ignoringDisable(true));

        controller
                .back()
                .and(controller.start())
                .and(isManualShoot)
                .and(isEnabled)
                .onFalse(
                        new InstantCommand(() -> conState = ControllerState.NORMAL)
                                .ignoringDisable(true));
        controller
                .back()
                .and(controller.start())
                .and(isManualShoot)
                .and(isEnabled.negate())
                .onFalse(
                        new InstantCommand(() -> conState = ControllerState.MANUAL_UNJAM)
                                .ignoringDisable(true));

        controller
                .back()
                .and(controller.start())
                .and(isManualUnjam)
                .and(isEnabled)
                .onFalse(
                        new InstantCommand(() -> conState = ControllerState.NORMAL)
                                .ignoringDisable(true));
        controller
                .back()
                .and(controller.start())
                .and(isManualUnjam)
                .and(isEnabled)
                .onFalse(
                        new InstantCommand(() -> conState = ControllerState.MANUAL_CLIMB)
                                .ignoringDisable(true));

        controller
                .back()
                .and(controller.start())
                .and(isManualClimb)
                .onFalse(
                        new InstantCommand(() -> conState = ControllerState.NORMAL)
                                .ignoringDisable(true));
    }
}
