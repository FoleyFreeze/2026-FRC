package frc.robot.subsystems.fuelvision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.Util2;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class FuelVision extends SubsystemBase {
    RobotContainer r;
    FuelVisionIO io;
    FuelVisionIOInputsAutoLogged inputs = new FuelVisionIOInputsAutoLogged();

    public static final boolean isDisabled = false;

    public static double oldestAllowedImage = 0.5;
    public static int bufferSize = (int) Math.ceil(oldestAllowedImage / 0.02);
    public static boolean disable = false;

    Translation2d lastNoteLocation;

    int badIdErr = 0;
    int badHeightErr = 0;
    int badXErr = 0;

    DoubleEntry rioTime =
            NetworkTableInstance.getDefault()
                    .getDoubleTopic("/Vision/RIO Time")
                    .getEntry(Timer.getFPGATimestamp());

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
        rioTime.set(inputs.now);

        if (Constants.currentMode == Mode.SIM) {
            // fake some camera data
            inputs.now = Timer.getTimestamp();
            inputs.seqNum = 53;
            inputs.rioTime = (float) (inputs.now - 0.02);
            inputs.imageTime = 0.08f;
            inputs.realTime =
                    (float) inputs.now
                            - ((float) inputs.now - inputs.rioTime) / 2.0f
                            - inputs.imageTime;
            inputs.fuelData = new FuelVisionData[5];
            inputs.fuelData[0] = new FuelVisionData();
            inputs.fuelData[0].distance = 19;
            inputs.fuelData[0].angle = 35;
            inputs.fuelData[0].orientation = 2;
            inputs.fuelData[0].amount = 1;
            inputs.fuelData[1] = new FuelVisionData();
            inputs.fuelData[1].distance = 34;
            inputs.fuelData[1].angle = -40;
            inputs.fuelData[1].orientation = 2;
            inputs.fuelData[1].amount = 1;
            inputs.fuelData[2] = new FuelVisionData();
            inputs.fuelData[2].distance = 55;
            inputs.fuelData[2].angle = 40;
            inputs.fuelData[2].orientation = 2;
            inputs.fuelData[2].amount = 1;
            inputs.fuelData[3] = new FuelVisionData();
            inputs.fuelData[3].distance = 80;
            inputs.fuelData[3].angle = 45;
            inputs.fuelData[3].orientation = 2;
            inputs.fuelData[3].amount = 1;
            inputs.fuelData[4] = new FuelVisionData();
            inputs.fuelData[4].distance = 100;
            inputs.fuelData[4].angle = 30;
            inputs.fuelData[4].orientation = 2;
            inputs.fuelData[4].amount = 1;
        }

        double ballSum = 0;
        for (int ball = 0; ball < inputs.fuelData.length; ball++) {
            ballSum += inputs.fuelData[ball].amount;
        }
        Logger.recordOutput("FuelVision/TotalBalls", ballSum);

        TimestampedPose2d now = new TimestampedPose2d();
        now.pose = r.drive.getPose();
        now.time = inputs.now;
        robotPoseBuffer.addFirst(now);
    }

    // angle to follow the wall at (dont be square, have some angle so the intake scrapes the wall)
    // this is calculated via tan-1(bumperwidth / (intakelength-bumperwidth))
    // i.e. what angle lets the corner of the bumper and the corner of the intake touch the wall
    // then back off a tad so we arent touching the wall
    public static final double wallAngle = Math.toRadians(18.5 - 4);
    public static final double wallBuffer = Units.inchesToMeters(2);
    public static final Translation2d wallOffset =
            new Translation2d(Constants.robotLength / 2, Constants.robotWidth / 2)
                    .rotateBy(Rotation2d.fromRadians(wallAngle))
                    .plus(new Translation2d(wallBuffer, wallBuffer));

    public static final double camFOV = Math.toRadians(90);
    public static final int numLines = 5;
    public static final double maxDeviation = Units.inchesToMeters(15);
    // for every meter of deviation, take x meters to get there
    public static final double maxDeviationSlope = 1;
    // TODO: measure real values
    public static final Translation2d camLocation =
            new Translation2d(
                    Constants.frameLength / 2 + Units.inchesToMeters(12),
                    Constants.frameWidth / 2 - Units.inchesToMeters(3));
    public static final Translation2d ctrFrtBumper =
            new Translation2d(Constants.robotLength / 2, 0);

    // rectangle bounding box (top left, top right, bot left, bot right)
    public double[][] fixedRects = new double[3][4];
    // line , x1y1x2y2
    public double[][] rays = new double[numLines][4];

    public double[] ballCount = new double[numLines];

    public enum Zone {
        BLUE,
        NEUTRAL,
        RED
    }

    public Pose2d getClosestFuel() {

        // if data is too old just return the robots current location
        if (Timer.getTimestamp() - inputs.realTime > oldestAllowedImage) return r.drive.getPose();

        // if no balls in image, return robots current location
        if (inputs.fuelData.length == 0) return r.drive.getPose();

        Pose2d botPose = getPoseAtCamTime(inputs.realTime);

        // simple version, just get the closest fuel
        FuelVisionData closeBall = inputs.fuelData[0];
        double minDist = closeBall.distance;
        for (int ball = 1; ball < inputs.fuelData.length; ball++) {
            if (inputs.fuelData[ball].distance < closeBall.distance) {
                closeBall = inputs.fuelData[ball];
                minDist = closeBall.distance;
            }
        }

        // transform to field pose
        double x = Units.inchesToMeters(closeBall.distance);
        double t = -Math.toRadians(closeBall.angle);
        Translation2d baseOffset = new Translation2d(x, x * Math.tan(t));
        Translation2d pos =
                baseOffset
                        .plus(camLocation)
                        .rotateBy(botPose.getRotation())
                        .plus(botPose.getTranslation());

        return new Pose2d(pos, botPose.getRotation());
    }

    public List<Pose2d> getFuelPath() {
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
        // TODO: just follow the edge paths?
        if (Timer.getTimestamp() - inputs.realTime > oldestAllowedImage) return new ArrayList<>();

        // step1
        Pose2d botPose = getPoseAtCamTime(inputs.realTime);
        Zone zone = findFieldZone(botPose);

        // set region bounds and ignore balls outside of them
        double[] rect = fixedRects[zone.ordinal()];

        // step zero, convert cam data to useful representations
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
                            .plus(camLocation)
                            .rotateBy(botPose.getRotation())
                            .plus(botPose.getTranslation());

            // if this ball is on the field add it to the list
            if (pos.getX() < rect[0]
                    && pos.getX() > rect[2]
                    && pos.getY() < rect[1]
                    && pos.getY() > rect[3]) {
                ballPos[ballPosLen] = pos;
                ballCounts[ballPosLen] = inputs.fuelData[ball].amount;
                ballPosLen++;
            }
        }

        // step 3 pick the best ray
        int bestLineIdx = 0;
        double mostBalls = 0;

        // keep track of which balls are available to each line
        int[] lineBallCount = new int[numLines];
        int[][] lineBallIdxs = new int[numLines][ballPosLen];

        Translation2d camFieldLoc =
                camLocation.rotateBy(botPose.getRotation()).plus(botPose.getTranslation());
        double lineWidth = camFOV / numLines;
        double startAngle = -camFOV / 2 + lineWidth / 2;
        for (int line = 0; line < numLines; line++) {
            // construct the line
            // the line is made of 2 points, one starting at the camera, and another at the first
            // fixed line it intersects
            // however, we can also represent the infinite line as the starting point (cam location)
            // and an angle (lineAngle) which will make some math easier
            double lineAngle = line * lineWidth + startAngle + botPose.getRotation().getRadians();
            ballCount[line] = 0;

            for (int ball = 0; ball < ballPosLen; ball++) {
                Translation2d pos = ballPos[ball];
                // the distance between a point and a line in 2d can be calculated via:
                // where the line is defined by a point and an angle (P, A)
                // where the point is (x, y)
                // then distance = cos(A)*(Py-y)-sin(A)*(Px-x)
                // see: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
                double dist =
                        Math.cos(lineAngle) * (camFieldLoc.getY() - pos.getY())
                                - Math.sin(lineAngle) * (camFieldLoc.getX() - pos.getX());
                if (Math.abs(dist) < maxDeviation) {
                    // track that this line will gather this many balls
                    ballCount[line] += ballCounts[ball];

                    // track that this line will need to remember to gather this ball
                    lineBallIdxs[line][lineBallCount[line]] = ball;
                    lineBallCount[line]++;
                }
            }

            if (ballCount[line] > mostBalls) {
                bestLineIdx = line;
                mostBalls = ballCount[line];
            }
        }

        // step4 construct the path
        // the path starts at the bot pose,
        // travels to each ball group close to the line
        // then to the intersection of the ray with the fixed boundary line
        // then follows the boundary to the corner
        // TODO: how to handle corners
        List<Pose2d> pathPoses = new ArrayList<>(lineBallCount[bestLineIdx]);

        ChassisSpeeds fieldVel = r.drive.getFieldVelocity();
        double speed = Util2.getScalarVel(fieldVel);
        if (speed > 0.2) {
            pathPoses.add(
                    new Pose2d(
                            botPose.getTranslation(),
                            new Rotation2d(
                                    fieldVel.vxMetersPerSecond, fieldVel.vyMetersPerSecond)));
        } else {
            pathPoses.add(botPose);
        }

        double rayAngle = bestLineIdx * lineWidth + startAngle + botPose.getRotation().getRadians();
        if (mostBalls == 0) {
            // if there are no balls, cast the ray along the robot angle
            rayAngle = botPose.getRotation().getRadians();
        }
        for (int ball = 0; ball < lineBallCount[bestLineIdx]; ball++) {
            // we want the robot path to be pointed in the direction of the line for each ball
            // gather
            // consider adding a little nudge towards the next ball if there is one though
            Pose2d targetPose =
                    new Pose2d(ballPos[lineBallIdxs[bestLineIdx][ball]], new Rotation2d(rayAngle));
            pathPoses.add(targetPose);
        }

        // find the intersection with the first fixed line
        Pose2d intersection =
                getIntersection(
                        fixedRects[zone.ordinal()], pathPoses.get(pathPoses.size() - 1), rayAngle);
        pathPoses.add(intersection);

        // continue until the corner
        if (intersection.getRotation().getCos() == 1) {
            pathPoses.add(new Pose2d(rect[0], intersection.getY(), intersection.getRotation()));
        } else if (intersection.getRotation().getSin() == 1) {
            pathPoses.add(new Pose2d(intersection.getX(), rect[1], intersection.getRotation()));
        } else if (intersection.getRotation().getCos() == -1) {
            pathPoses.add(new Pose2d(rect[2], intersection.getY(), intersection.getRotation()));
        } else {
            pathPoses.add(new Pose2d(intersection.getX(), rect[3], intersection.getRotation()));
        }

        return pathPoses;
    }

    // uses a modified version of the Liang Barsky clipping algo
    private Pose2d getIntersection(double[] rect, Pose2d botPose, double lineAngle) {
        double startX = botPose.getX();
        double startY = botPose.getY();

        // force start points into the rect
        if (startX > rect[0]) {
            startX = rect[0] - 0.01;
        } else if (startX < rect[2]) {
            startX = rect[2] + 0.01;
        }
        if (startY > rect[1]) {
            startY = rect[1] - 0.01;
        } else if (startY < rect[3]) {
            startY = rect[3] + 0.01;
        }

        // create a second point for our line. make it long enough that it will definitely hit one
        // of the lines of the bounding rectangle
        double offset = FieldConstants.fieldWidth * 2;
        double endX = startX + Math.cos(lineAngle) * offset;
        double endY = startY + Math.sin(lineAngle) * offset;

        double p1 = -(endX - startX);
        double p2 = -p1;
        double p3 = -(endY - startY);
        double p4 = -p3;

        double q1 = startX - rect[2];
        double q2 = rect[0] - startX;
        double q3 = startY - rect[3];
        double q4 = rect[1] - startY;

        if ((p1 == 0 && q1 < 0)
                || (p2 == 0 && q1 < 0)
                || (p3 == 0 && q3 < 0)
                || (p4 == 0 && q4 < 0)) {
            // line does not intersect
            if (lineAngle == 0 || lineAngle == Math.PI) {
                // we tried twice and it still doesnt intersect
                return new Pose2d(botPose.getTranslation(), Rotation2d.fromRadians(lineAngle));
            }
            return getIntersection(rect, botPose, botPose.getRotation().getCos() > 0 ? 0 : Math.PI);
        }

        // only looking for exit points
        double exitT = 1;
        int xidx = -1;
        int yidx = -1;
        if (p1 != 0) { // means there is x movement
            if (p1 < 0) { // line will intersect +x bound
                exitT = q2 / p2;
                xidx = 0;
                yidx = -1;
            } else { // line will intersect -x bound
                exitT = q1 / p1;
                xidx = 2;
                yidx = -1;
            }
        }
        if (p3 != 0) { // there is y movement
            if (p3 < 0) { // line will intersect +y bound
                double r = q4 / p4;
                if (r < exitT) {
                    exitT = r;
                    xidx = -1;
                    yidx = 1;
                }
            } else { // line will intersect -y bound
                double r = q3 / p3;
                if (r < exitT) {
                    exitT = r;
                    xidx = -1;
                    yidx = 3;
                }
            }
        }

        double x = startX + (endX - startX) * exitT;
        double y = startY + (endY - startY) * exitT;

        // determine the remaining direction via dot product
        double x1 = startX - x;
        double y1 = startY - y;
        double x2, y2, dot, angle;
        if (xidx == -1) {
            // intersecting a vertical bound
            x2 = rect[0] - x;
            y2 = rect[yidx] - y;
            dot = x1 * x2 + y1 * y2;
            if (dot > 0) {
                // start points towards rect[0,y] so we want to point robot the opposite
                angle = Math.PI;
            } else {
                angle = 0;
            }
        } else /*if(yidx == -1)*/ {
            // intersecting a horizontal bound
            x2 = rect[xidx] - x;
            y2 = rect[1] - y;
            dot = x1 * x2 + y1 * y2;
            if (dot > 0) {
                // start points towards rect[x,1] so we want to point robot the opposite
                angle = -Math.PI / 2;
            } else {
                angle = Math.PI / 2;
            }
        }

        return new Pose2d(x, y, Rotation2d.fromRadians(angle));
    }

    public Zone findFieldZone(Pose2d botPose) {
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
            return past.pose;
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
        // BLUE
        setRect(
                fixedRects[0],
                FieldConstants.HorizontalLines.starting,
                FieldConstants.fieldWidth,
                0,
                0);

        // NEUTRAL
        setRect(
                fixedRects[1],
                FieldConstants.HorizontalLines.neutralEnd,
                FieldConstants.fieldWidth,
                FieldConstants.HorizontalLines.neutralStart,
                0);

        // RED
        setRect(
                fixedRects[2],
                FieldConstants.fieldLength,
                FieldConstants.fieldWidth,
                mirrorX(FieldConstants.HorizontalLines.starting),
                0);
    }

    // top left corner, bot right corner
    private void setRect(double[] line, double x1, double y1, double x2, double y2) {
        double norm = wallOffset.getNorm();
        line[0] = x1 - norm;
        line[1] = y1 - norm;
        line[2] = x2 + norm;
        line[3] = y2 + norm;
    }
}
