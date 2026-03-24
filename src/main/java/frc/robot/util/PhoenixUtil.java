// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.AutoLogOutput;

public class PhoenixUtil {
    @AutoLogOutput(key = "FailedConfigs")
    public static int numFailures = 0;

    /** Attempts to run the command until no error is produced. */
    public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        var error = StatusCode.OK;
        boolean success = false;
        for (int i = 0; i < maxAttempts; i++) {
            error = command.get();
            if (error.isOK()) {
                success = true;
                break;
            }
        }

        if (success == false) {
            System.out.println("-----------------------------------------------");
            System.out.println("-----------------------------------------------");
            System.out.println("Config apply failed with: " + error.toString());
            System.out.println("-----------------------------------------------");
            System.out.println("-----------------------------------------------");
            numFailures++;
        }
    }

    public static double[] getSimulationOdometryTimeStamps() {
        final double[] odometryTimeStamps =
                new double[SimulatedArena.getSimulationSubTicksIn1Period()];
        for (int i = 0; i < odometryTimeStamps.length; i++) {
            odometryTimeStamps[i] =
                    Timer.getFPGATimestamp()
                            - 0.02
                            + i * SimulatedArena.getSimulationDt().in(Seconds);
        }

        return odometryTimeStamps;
    }
}
