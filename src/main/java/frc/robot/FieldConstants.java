package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

// field according to the blue side
// inspired by 6328's FieldConstants class
// note all dimensions are in meters
public class FieldConstants {
    public static final AprilTagFieldLayout tagLayout;
    public static final double fieldLength;
    public static final double fieldWidth;

    static {
        tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        fieldLength = tagLayout.getFieldLength();
        fieldWidth = tagLayout.getFieldWidth();
    }

    public static class HorizontalLines {
        public static final double center = fieldLength / 2;
        // tag 26 is the front of the hub, right above the starting line
        public static final double starting = tagLayout.getTagPose(26).get().getX();
        public static final double neutralStart = center - Units.inchesToMeters(120);
        public static final double neutralEnd = center + Units.inchesToMeters(120);
    }

    public static class VerticalLines {
        public static final double center = fieldWidth / 2;
    }

    public static class Hub {
        public static final double width = Units.inchesToMeters(47);
        public static final double goalWidth = Units.inchesToMeters(41.7);
        public static final double height = Units.inchesToMeters(72);
        public static final Translation2d center =
                new Translation2d(HorizontalLines.starting + width / 2, VerticalLines.center);
        public static final Translation2d backLeft =
                center.plus(new Translation2d(width / 2, width / 2));
        public static final Translation2d backRight =
                center.plus(new Translation2d(width / 2, -width / 2));
        public static final Translation2d frontRight =
                center.minus(new Translation2d(width / 2, -width / 2));
        public static final Translation2d frontLeft =
                center.minus(new Translation2d(width / 2, -width / 2));
        // consider adding corners (topLeftCorner...)
    }

    public static class LeftBump {}

    public static class RightBump {}

    public static class LeftTrench {}

    public static class RightTrench {}

    public static class Tower {
        public static final double halfWidth = Units.inchesToMeters(49.25 / 2.0);
        public static final double depth = Units.inchesToMeters(45 - 3.25);
        public static final Translation2d center =
                new Translation2d(depth, tagLayout.getTagPose(31).get().getY());
        public static Translation2d towerLeftFront =
                new Translation2d(depth, center.getY() + halfWidth);
        public static Translation2d towerRightFront =
                new Translation2d(depth, center.getY() - halfWidth);
        public static Pose2d towerLeftFrontPose2d =
                new Pose2d(depth, center.getY() + halfWidth, Rotation2d.fromDegrees(0));
        public static Pose2d towerRightFrontPose2d =
                new Pose2d(depth, center.getY() - halfWidth, Rotation2d.fromDegrees(0));
    }

    public static class Depot {}

    public static class Outpost {}

    public static class Locations {
        public static Translation2d passLeft =
                new Translation2d(Units.feetToMeters(3), fieldWidth - Units.feetToMeters(3));
        public static Translation2d passRight =
                new Translation2d(Units.feetToMeters(3), Units.feetToMeters(3));
        public static Pose2d locationHubShoot =
                new Pose2d(
                        Hub.frontLeft.getX() - Constants.robotLength / 2,
                        Hub.center.getY(),
                        Rotation2d.fromDegrees(0));
        public static Pose2d locationClimbShoot =
                new Pose2d(
                        Tower.center.getX() + Constants.robotLength / 2,
                        Tower.center.getY(),
                        Rotation2d.fromDegrees(0));
    }

    // flip a red coord to blue or blue to red
    public static Translation2d flip(Translation2d pos) {
        if (pos.getX() < HorizontalLines.center) {
            return new Translation2d(pos.getX() + HorizontalLines.center, fieldWidth - pos.getY());
        } else {
            return new Translation2d(pos.getX() - HorizontalLines.center, fieldWidth - pos.getY());
        }
    }

    // flip if on red
    public static Translation2d flipIfRed(Translation2d pos) {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            return flip(pos);
        } else {
            return pos;
        }
    }
}
