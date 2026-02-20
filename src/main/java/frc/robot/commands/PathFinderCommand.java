package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import java.util.function.Supplier;

public class PathFinderCommand extends Command {
    RobotContainer r;
    private Supplier<Pose2d> poseSupplier;
    private Command c; // captured pathfinding command

    private final PathConstraints globalConstraints =
            new PathConstraints(3.5, 4.4, 12, 15); // TODO: make faster later

    public PathFinderCommand(RobotContainer r, Supplier<Pose2d> poseSupplier) {
        this.r = r;
        this.poseSupplier = poseSupplier;
    }

    @Override
    public void initialize() {
        // step1: where go?
        Pose2d target = poseSupplier.get();

        // step2: create the command via path planner
        c = AutoBuilder.pathfindToPose(target, globalConstraints);
        c.initialize();
    }

    @Override
    public void execute() {
        c.execute();
    }

    @Override
    public void end(boolean interrupted) {
        c.end(interrupted);
        r.drive.runVelocity(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return c.isFinished();
    }
}
