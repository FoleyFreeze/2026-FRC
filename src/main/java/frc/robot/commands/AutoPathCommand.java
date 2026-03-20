package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AutoPathCommand extends Command {
    RobotContainer r;
    private Command c; // captured command
    private PathPlannerPath path;

    // this is for the pathfinding portion alone
    private final PathConstraints globalConstraints = new PathConstraints(3.5, 4.4, 12, 15);

    public AutoPathCommand(RobotContainer r, PathPlannerPath path) {
        this.r = r;
        this.addRequirements(r.drive);
        this.path = path;
    }

    @Override
    public void initialize() {
        c = AutoBuilder.pathfindThenFollowPath(path, globalConstraints);
        c.initialize();
    }

    @Override
    public void execute() {
        // if(notFollowingPathWell()){
        //     c.end(true);
        //     regeneratePath();
        //     c.initialize();
        // }

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
