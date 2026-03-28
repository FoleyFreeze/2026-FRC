// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "limelight-turret";
    public static String camera1Name = "limelight-left";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToCamera0 =
            new Transform3d(
                    -Constants.frameLength / 2.0 + Units.inchesToMeters(7.25),
                    Constants.frameWidth / 2.0 - Units.inchesToMeters(12.25 + 0.340),
                    Units.inchesToMeters(21.3125 - 0.1585),
                    new Rotation3d(0.0, Math.toRadians(25), Math.toRadians(90)));
    public static Transform3d robotToCamera1 =
            new Transform3d(
                    -Constants.frameLength / 2 + Units.inchesToMeters(4.23),
                    Constants.frameWidth / 2 - Units.inchesToMeters(1.57),
                    Units.inchesToMeters(8.779),
                    new Rotation3d(0.0, Math.toRadians(30), Math.toRadians(90)));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
            new double[] {
                2.0, // Camera 0
                6.0 // Camera 1
            };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.35; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor =
            Double.POSITIVE_INFINITY; // No rotation data available
}
