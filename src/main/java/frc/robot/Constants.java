// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    // with bumpers
    public static final double robotLength =
            Units.inchesToMeters(32 + (1.0 / 16.0)); // front and back X
    public static final double robotWidth = Units.inchesToMeters(34.5625); // left and right Y

    // without bumpers
    public static final double frameLength = Units.inchesToMeters(26); // front and back X
    public static final double frameWidth = Units.inchesToMeters(28.5); // left and right Y

    public static final Translation2d shooterLocOnBot =
            new Translation2d(
                    (frameLength / 2) - Units.inchesToMeters(22),
                    (-frameWidth / 2) + Units.inchesToMeters(13));
    public static final double turretAngleOffset = 0;
    public static final double maximumTurretAngle = 400;
}
