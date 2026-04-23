package frc.robot.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.ShooterCommands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class BlineAutos {

    RobotContainer r;
    FollowPath.Builder pathBuilder;
    boolean useDynamicVel = false;

    public BlineAutos(RobotContainer r) {
        this.r = r;
        pathBuilder =
                new FollowPath.Builder(
                                r.drive, // Subsystem requirement
                                r.drive::getPose, // Supplier<Pose2d>
                                // Supplier<ChassisSpeeds> (robot-relative)
                                r.drive::getChassisSpeeds,
                                // Consumer<ChassisSpeeds>  (robot-relative)
                                this::runDynamicVel,
                                // translation — minimizes remaining distance
                                new PIDController(5.0, 0.0, 0.0),
                                // rotation    — minimizes heading error
                                new PIDController(3.0, 0.0, 0.0),
                                // cross-track — minimizes perpendicular deviation
                                new PIDController(2.0, 0.0, 0.0))
                        // auto-flip when on the red alliance
                        .withDefaultShouldFlip();
        // .withPoseReset(r.drive::resetPose); // reset odometry at each path's start pose

        FollowPath.setDoubleLoggingConsumer(p -> Logger.recordOutput(p.getFirst(), p.getSecond()));
        FollowPath.setBooleanLoggingConsumer(p -> Logger.recordOutput(p.getFirst(), p.getSecond()));
        FollowPath.setPoseLoggingConsumer(p -> Logger.recordOutput(p.getFirst(), p.getSecond()));
        FollowPath.setTranslationListLoggingConsumer(
                p -> Logger.recordOutput(p.getFirst(), p.getSecond()));
    }

    public void buildAutos(LoggedDashboardChooser<Command> autoChooser) {
        autoChooser.addOption("left_pass_repeat", buildPass("trench_pass_repeat", true));
        autoChooser.addOption("right_pass_repeat", buildPass("trench_pass_repeat", false));
        autoChooser.addOption("left_pass_wait", buildPass("trench_pass_wait", true));
        autoChooser.addOption("right_pass_wait", buildPass("trench_pass_wait", false));
    }

    public Command buildPass(String name, boolean flipLR) {
        double initialShootWait = 1.2;
        double passDelay = 1.5;
        useDynamicVel = true;

        SequentialCommandGroup sequence = new SequentialCommandGroup();
        // first drop the intake as fast as possible
        sequence.addCommands(
                ShooterCommands.smartShoot(r, FieldConstants.Hub.center)
                        .alongWith(new InstantCommand(r.intake::extend, r.intake))
                        .withTimeout(initialShootWait)
                        .finallyDo(
                                () -> {
                                    r.shooter.stopAll().execute();
                                    r.spindexter.stop().execute();
                                    r.intake.reallyExtend();
                                    r.intake.stopIntake().initialize();
                                }));

        // drive the profile while intaking and passing after a delay
        Path passPath = new Path(name);
        if(flipLR){
            passPath.mirror();
        }
        ParallelCommandGroup parallelGroup =
                new ParallelCommandGroup(
                        pathBuilder.build(passPath),
                        r.intake.smartIntake(),
                        ShooterCommands.smartShoot(r, FieldConstants.Locations.passLeft)
                                .beforeStarting(new WaitCommand(passDelay)));
        sequence.addCommands(parallelGroup);

        // map the initial pose to the auton command so we can reset pose on selection
        Runnable run =
                () -> {
                    Pose2d pose = passPath.getStartPose(new Rotation2d());
                    r.drive.setPose(FieldConstants.flipIfRed(pose));
                };
        r.pathAutos.pathMap.put(sequence, run);
        return sequence;
    }

    public void runDynamicVel(ChassisSpeeds speeds) {
        if (useDynamicVel) {
            double maxVel = 1.5;
            double factor = r.intake.calcIntakeSpeed();
            // note that this intentionally does not scale rotational velocity
            double vx = speeds.vxMetersPerSecond * factor;
            speeds.vxMetersPerSecond = MathUtil.clamp(vx, -maxVel, maxVel);
            double vy = speeds.vyMetersPerSecond * factor;
            speeds.vyMetersPerSecond = MathUtil.clamp(vy, -maxVel, maxVel);
            r.drive.runVelocity(speeds.times(factor));
        } else {
            r.drive.runVelocity(speeds);
        }
    }
}
