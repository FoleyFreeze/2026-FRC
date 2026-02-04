package frc.robot.subsystems.stats;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class StatsSubsystem extends SubsystemBase {
    Stats stats = new Stats();

    @Override
    public void periodic() {
        stats.totalTravelDist += 1;
        stats.totalClimbs += 0.001;
        stats.totalTurretRotations += 0.02;
        Logger.recordOutput("Stats", stats);
    }
}
