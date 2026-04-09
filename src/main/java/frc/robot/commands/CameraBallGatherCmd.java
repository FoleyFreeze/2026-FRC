package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.fuelvision.FuelVision;
import frc.robot.subsystems.fuelvision.FuelVision.Zone;
import frc.robot.util.Util2;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class CameraBallGatherCmd extends Command {
    RobotContainer r;
    private Command c; // captured command
    private PathPlannerPath path;

    private final PathConstraints globalConstraints = new PathConstraints(2, 0.5, 1, 2);

    public CameraBallGatherCmd(RobotContainer r) {
        this.r = r;
        this.addRequirements(r.drive);
    }

    @Override
    public void initialize() {
        c = null;
        try {
            List<Pose2d> pathPoses = r.fuelVision.getFuelPath();
            Logger.recordOutput("FuelVision/FailReason", r.fuelVision.failReason);
            if (pathPoses.isEmpty()) {
                // exit early if path failed
                c = null;
                return;
            }
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(pathPoses);

            List<RotationTarget> rotationTargets = new ArrayList<>(pathPoses.size());

            // note we are starting from 1 to skip the first pose (the robot starting position)
            // ending at -2 to skip the final wall ride
            for (int i = 1; i < pathPoses.size() - 2; i++) {
                Pose2d p = pathPoses.get(i);
                rotationTargets.add(new RotationTarget(i, p.getRotation()));
            }

            Zone z = r.fuelVision.findFieldZone(pathPoses.get(0));
            double[] rect = r.fuelVision.fixedRects[z.ordinal()];
            Translation2d mid = new Translation2d((rect[0] + rect[2]) / 2, (rect[1] + rect[3]) / 2);
            Rotation2d angle1 =
                    pathPoses.get(pathPoses.size() - 2).getTranslation().minus(mid).getAngle();
            Rotation2d angle2 =
                    pathPoses.get(pathPoses.size() - 1).getTranslation().minus(mid).getAngle();
            double result = angle2.minus(angle1).getRadians();
            double wallAngle = FuelVision.wallAngle;
            if (result > 0) {
                wallAngle = -wallAngle;
            }

            // offset the final angle by the wall angle
            rotationTargets.add(
                    new RotationTarget(
                            pathPoses.size() - 2,
                            pathPoses
                                    .get(pathPoses.size() - 2)
                                    .getRotation()
                                    .plus(Rotation2d.fromRadians(wallAngle))));

            PathPlannerPath path =
                    new PathPlannerPath(
                            waypoints,
                            rotationTargets,
                            List.of(),
                            List.of(),
                            List.of(),
                            globalConstraints,
                            // we do want to provide this so that we only generate one trajectory
                            // instead of 2
                            new IdealStartingState(
                                    Util2.getScalarVel(r.drive.getChassisSpeeds()),
                                    r.drive.getRotation()),
                            new GoalEndState(
                                    0, rotationTargets.get(rotationTargets.size() - 1).rotation()),
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
