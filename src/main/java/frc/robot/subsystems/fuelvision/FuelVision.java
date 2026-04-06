package frc.robot.subsystems.fuelvision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.Timer;
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

    public static final double camFOV = Math.toRadians(90);
    public static final int numLines = 5;
    public static final double maxDeviation = Units.inchesToMeters(24);
    // for every meter of deviation, take x meters to get there
    public static final double maxDeviationSlope = 1;
    // TODO: measure real values
    public static final Translation2d camLocation =
            new Translation2d(
                    Constants.frameLength / 2 + Units.inchesToMeters(12),
                    Constants.frameWidth / 2 - Units.inchesToMeters(3));
    public static final Translation2d ctrFrtBumper =
            new Translation2d(
                    Constants.frameLength / 2 + (Constants.robotLength - Constants.frameLength) / 2,
                    0);

    // line (RFLB), x1y1x2y2, apply X offset for region
    public double[][] fixedLines = new double[4 * 2][4];
    // line , x1y1x2y2
    public double[][] rays = new double[numLines][4];

    public double[] ballCount = new double[numLines];

    public enum Zone {
        BLUE,
        NEUTRAL,
        RED
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

        // abort if data is too old
        if (Timer.getTimestamp() - inputs.realTime > 0.5) return;

        //step1
        Pose2d botPose = getPoseAtCamTime(inputs.realTime);
        Zone zone = findFieldZone(botPose);

        //set region bounds and ignore balls outside of them
        double xmax,ymax,xmin,ymin;
        ymin = 0;
        ymax = FieldConstants.fieldWidth;
        switch(zone){
            case BLUE:
            default:
            xmin = 0;
            xmax = FieldConstants.HorizontalLines.starting;
            break;
            case NEUTRAL:
            xmin = FieldConstants.HorizontalLines.neutralStart;
            xmax = FieldConstants.HorizontalLines.neutralEnd;
            break;
            case RED:
            xmin = mirrorX(FieldConstants.HorizontalLines.starting);
            xmax = FieldConstants.fieldLength;
            break;
        }

        //step zero, convert cam data to useful representations
        int ballPosLen = 0;
        Translation2d[] ballPos = new Translation2d[inputs.fuelData.length];
        int[] ballCounts = new int[inputs.fuelData.length];
        for (int ball = 0; ball < inputs.fuelData.length; ball++) {
            // data provided as x, theta
            // turn into field xy via adding to robot position
            // note cam angles are inverted (cw positive) so flip them
            // TODO: is cam data robot or camera relative (I think its center front bumper, which is
            // neither)
            double x = Units.inchesToMeters(inputs.fuelData[ball].distance);
            double t = -Math.toRadians(inputs.fuelData[ball].angle);
            Translation2d baseOffset = new Translation2d(x, x * Math.tan(t));
            // construct the final field position by:
            // 1) adding the camera relative location
            // 2) rotating by the robot angle
            // 3) adding the robot field position
            Translation2d pos = 
                    baseOffset
                            .plus(ctrFrtBumper)
                            .rotateBy(botPose.getRotation())
                            .plus(botPose.getTranslation());
            
            //if this ball is on the field add it to the list
            if(pos.getX() < xmax && 
               pos.getX() > xmin && 
               pos.getY() < ymax && 
               pos.getY() > ymin){
                ballPos[ballPosLen] = pos;
                ballCounts[ballPosLen] = inputs.fuelData[ball].amount;
                ballPosLen++;
            }
        }

        //step 3 pick the best ray
        int bestLineIdx = 0;
        double mostBalls = 0;

        Translation2d camFieldLoc = camLocation.rotateBy(botPose.getRotation()).plus(botPose.getTranslation());
        double lineWidth = camFOV / numLines;
        double startAngle = -camFOV/2 + lineWidth/2;
        for (int line = 0; line < numLines; line++) {
            //construct the line
            //the line is made of 2 points, one starting at the camera, and another at the first fixed line it intersects
            double lineAngle = line*lineWidth + startAngle + botPose.getRotation().getRadians();
            ballCount[line] = 0;

            for (int ball = 0; ball < ballPosLen; ball++) {
                Translation2d pos = ballPos[ball];
                //the distance between a point and a line in 2d can be calculated via:
                // where the line is defined by a point and an angle (P, A)
                // where the point is (x, y)
                // then distance = cos(A)*(Py-y)-sin(A)*(Px-x)
                //see: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
                double dist = Math.cos(lineAngle)*(camFieldLoc.getY()-pos.getY()) - 
                              Math.sin(lineAngle)*(camFieldLoc.getX()-pos.getX());
                if(Math.abs(dist) < maxDeviation){
                    ballCount[line] += ballCounts[ball];
                }
            }

            if(ballCount[line] > mostBalls){
                bestLineIdx = line;
                mostBalls = ballCount[line];
            }
        }

        //step4 construct the path
        
    }

    private Zone findFieldZone(Pose2d botPose) {
        double x = botPose.getX();

        if (x < FieldConstants.HorizontalLines.starting) {
            // in blue alliance zone
            return Zone.BLUE;
        } else if (x < FieldConstants.HorizontalLines.neutralStart) {
            // between neutral and blue, select based on angle
            // using cosine since its positive for angles between -90 and 90 (i.e. pointed forward)
            if (botPose.getRotation().getCos() > 0) {
                return Zone.NEUTRAL;
            } else {
                return Zone.BLUE;
            }
        } else if (x < FieldConstants.HorizontalLines.neutralEnd) {
            // in neutral zone
            return Zone.NEUTRAL;
        } else if (x < mirrorX(FieldConstants.HorizontalLines.starting)) {
            // between neutral and red, select based on angle
            if (botPose.getRotation().getCos() > 0) {
                return Zone.RED;
            } else {
                return Zone.NEUTRAL;
            }
        } else {
            // in red zone
            return Zone.RED;
        }
    }

    private double mirrorX(double x) {
        return FieldConstants.fieldLength - x;
    }

    private Pose2d getPoseAtCamTime(double time) {
        TimestampedPose2d future = robotPoseBuffer.getFirst();
        TimestampedPose2d past = robotPoseBuffer.getLast();

        // if data is too old, dont use any image data
        if (past.time > time) {
            return r.drive.getPose();
        }

        for (int i = 1; i < robotPoseBuffer.size(); i++) {
            past = robotPoseBuffer.get(i);
            if (time < past.time) {
                future = past;
            } else {
                break;
            }
        }

        // interp between past and future
        double t = MathUtil.inverseInterpolate(past.time, future.time, time);
        return past.pose.interpolate(future.pose, t);
    }

    private void makeFixedLines() {
        // RFLB (right,far,left,back) rotate around ccw
        // x1y1x2y2
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

        setLine(
                fixedLines[4],
                FieldConstants.HorizontalLines.neutralStart,
                0,
                FieldConstants.HorizontalLines.neutralEnd,
                0);
        setLine(
                fixedLines[5],
                FieldConstants.HorizontalLines.neutralEnd,
                0,
                FieldConstants.HorizontalLines.neutralEnd,
                FieldConstants.fieldWidth);
        setLine(
                fixedLines[6],
                FieldConstants.HorizontalLines.neutralStart,
                FieldConstants.fieldWidth,
                FieldConstants.HorizontalLines.neutralEnd,
                FieldConstants.fieldWidth);
        setLine(
                fixedLines[7],
                FieldConstants.HorizontalLines.neutralStart,
                0,
                FieldConstants.HorizontalLines.neutralStart,
                FieldConstants.fieldWidth);
    }

    private void setLine(double[] line, double x1, double y1, double x2, double y2) {
        line[0] = x1;
        line[1] = y1;
        line[2] = x2;
        line[3] = y2;
    }
}
