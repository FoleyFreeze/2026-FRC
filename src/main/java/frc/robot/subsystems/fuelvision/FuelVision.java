package frc.robot.subsystems.fuelvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class FuelVision extends SubsystemBase {
    RobotContainer r;
    FuelVisionIO io;
    FuelVisionIOInputsAutoLogged inputs = new FuelVisionIOInputsAutoLogged();

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
}
