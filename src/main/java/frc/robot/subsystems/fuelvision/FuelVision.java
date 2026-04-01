package frc.robot.subsystems.fuelvision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class FuelVision extends SubsystemBase {
    RobotContainer r;
    FuelVisionIO io;
    FuelVisionIOInputsAutoLogged inputs = new FuelVisionIOInputsAutoLogged();

    public static final boolean isDisabled = false;

    public static int bufferSize = 8;
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

        makeFixedLines();
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

    public static final double camFOV = 90;
    public static final int numLines = 5;
    public static final double maxDeviation = Units.inchesToMeters(24);
    public static final double maxDeviationSlope =
            1; // i.e. for every meter of deviation, take x meters to get there
    // TODO: measure real values
    public static final Translation2d camLocation =
            new Translation2d(
                    Units.inchesToMeters(Constants.frameLength / 2 + 12),
                    Constants.frameWidth / 2 - 3);

    // line (RFLB), x1y1x2y2, apply X offset for region
    public double[][] fixedLines = new double[4][4];

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

        // step1 - where is the robot, our zone, neutral zone, their zone
        //       - need to account for when the robot is on the bump or in the trench
        //       - we should take the zone its pointed at in those cases
        // step2 - cast rays from the camera center (this is offset from the robot)
        //       - determine equations for these lines
        // step3 - count balls that intersect these rays, until the ray intersects a fixed line
        //       - if the robot is in a bump zone, dont intersect the first fixed line
        //       - apply some kind of logic to ensure that if we sidestep for a ball one way we cant
        // also go the other way
        //       - once a ray intersects a fixed line, count the balls along that fixed line
        // step4 - pick the ray with the most balls and construct a path that will travel that line
        // gathering them

        Pose2d botPose = getPoseAtCamTime(inputs.imageTime);
    }

    private Pose2d getPoseAtCamTime(double time){
        TimestampedPose2d future = robotPoseBuffer.getFirst();
        TimestampedPose2d past = robotPoseBuffer.getLast();

        //if data is too old, dont use any image data
        if(past.time > time){
            return r.drive.getPose();
        }
        
        for(int i=1;i<robotPoseBuffer.size();i++){
            past = robotPoseBuffer.get(i);
            if(time < past.time){
                future = past;
            } else {
                break;
            }
        }

        //interp between past and future
        double t = MathUtil.inverseInterpolate(past.time, future.time, time);
        return past.pose.interpolate(future.pose, t);
    }

    private void makeFixedLines() {
        setLine(fixedLines[0], 0, 0, FieldConstants.HorizontalLines.starting, 0);
        setLine(
                fixedLines[1],
                FieldConstants.HorizontalLines.starting,
                0,
                FieldConstants.HorizontalLines.starting,
                FieldConstants.fieldWidth);
        setLine(
                fixedLines[2],
                0,
                FieldConstants.fieldWidth,
                FieldConstants.HorizontalLines.starting,
                FieldConstants.fieldWidth);
        setLine(fixedLines[3], 0, 0, 0, FieldConstants.fieldWidth);
    }

    private void setLine(double[] line, double x1, double y1, double x2, double y2) {
        line[0] = x1;
        line[1] = y1;
        line[2] = x2;
        line[3] = y2;
    }
}
