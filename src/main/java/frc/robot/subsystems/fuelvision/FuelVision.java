package frc.robot.subsystems.fuelvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class FuelVision extends SubsystemBase {
    RobotContainer r;
    FuelVisionIO io;
    FuelVisionIOInputsAutoLogged inputs = new FuelVisionIOInputsAutoLogged();

    public static final boolean isDisabled = false;

    public static int bufferSize = 1;
    public static boolean disable = false;

    Translation2d lastNoteLocation;

    int badIdErr = 0;
    int badHeightErr = 0;
    int badXErr = 0;

    DoubleEntry rioTime =
            NetworkTableInstance.getDefault().getDoubleTopic("/Vision/RIO Time").getEntry(0);

    public static class TimestampedPose2d {
        Pose2d pose;
        double time;
    }

    CircularBuffer<TimestampedPose2d> robotPoseBuffer;

    public static FuelVision create(RobotContainer r) {
        if (isDisabled) {
            return new FuelVision(r);
        }

        switch (Constants.currentMode) {
            case REAL:
                return new FuelVision(r);
            case SIM:
                return new FuelVision(r);
            default:
                return new FuelVision(r);
        }
    }

    public FuelVision(RobotContainer r) {
        this.r = r;

        if (Robot.isReal() && !disable) {
            io = (FuelVisionIO) new FuelVisionIO_HW();
        } else {
            io = new FuelVisionIO() {};
        }

        robotPoseBuffer = new CircularBuffer<>(bufferSize);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("FuelVision", inputs);

        TimestampedPose2d now = new TimestampedPose2d();
        now.pose = r.drive.getPose();
        now.time = inputs.now;
        robotPoseBuffer.addFirst(now);
    }

    public void getFuelPath() {
        // return some kind of path that travels to the largest ball grouping and strafes into any
        // smaller groups on the way
        // we are thinking in straight-ish lines, either rays from the camera pov or lines along the
        // field borders (plus bump/hub sides)
        // estimate the ball count along these lines, and then drive the one with the most balls

        // define my lines
        // 8 field relative lines, 4 field borders, + 2 sides of red bump/hub + 2 sides of blue
        // bump/hub
        // N camera fov lines
        // only allocate N lines, start at cam pov and when it intersects a field line, continue
        // along that line
        // for each ball group
        // convert to field coords
        // for each line

        // keep track of the line with the most balls
        // drive it
    }
}
