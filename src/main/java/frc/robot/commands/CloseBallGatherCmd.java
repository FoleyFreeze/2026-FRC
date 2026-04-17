package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.Util2;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class CloseBallGatherCmd extends Command {
    RobotContainer r;
    private Command c; // captured command
    private final PathConstraints globalConstraints = new PathConstraints(2, 0.5, 1, 2);

    public CloseBallGatherCmd(RobotContainer r) {
        this.r = r;
        this.addRequirements(r.drive);
    }

    @Override
    public void initialize() {
        c = null;
        try {
            Pose2d closestFuel = r.fuelVision.getClosestFuel();
            Logger.recordOutput("FuelVision/FailReason", r.fuelVision.failReason);
            if (closestFuel == null) {
                // exit early if path failed
                c = null;
                return;
            }
            List<Waypoint> waypoints =
                    PathPlannerPath.waypointsFromPoses(r.drive.getPose(), closestFuel);

            PathPlannerPath path =
                    new PathPlannerPath(
                            waypoints,
                            List.of(),
                            List.of(),
                            List.of(),
                            List.of(),
                            globalConstraints,
                            // we do want to provide this so that we only generate one trajectory
                            // instead of 2
                            new IdealStartingState(
                                    Util2.getScalarVel(r.drive.getChassisSpeeds()),
                                    r.drive.getRotation()),
                            new GoalEndState(globalConstraints.maxVelocity(), r.drive.getRotation()),
                            false);
            path.preventFlipping = true;

            c = AutoBuilder.followPath(path);
            c.initialize();

        } catch (Exception e) {
            System.out.println("Error generating fuel path:");
            e.printStackTrace();
        }
    }

    @Override
    public void execute() {
        // if(notFollowingPathWell()){
        //     c.end(true);
        //     regeneratePath();
        //     c.initialize();
        // }
        if (c != null) c.execute();
    }

    @Override
    public void end(boolean interrupted) {
        if (c != null) c.end(interrupted);
        r.drive.runVelocity(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        if (c != null) return c.isFinished();
        else return true;
    }
}
