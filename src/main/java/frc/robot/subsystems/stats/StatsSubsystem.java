package frc.robot.subsystems.stats;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class StatsSubsystem extends SubsystemBase {
    RobotContainer r;
    Stats stats = new Stats();

    double botRadius = Drive.getModuleTranslations()[0].getNorm();

    double prevtime;
    boolean rpmDecreasing;

    public StatsSubsystem(RobotContainer r) {
        this.r = r;
        prevtime = Timer.getFPGATimestamp();
    }

    @Override
    public void periodic() {
        ChassisSpeeds speed = r.drive.getChassisSpeeds();
        double time = Timer.getFPGATimestamp();
        double dt = time - prevtime;
        prevtime = time;
        stats.totalTravelDist +=
                speed.vxMetersPerSecond * dt
                        + speed.vyMetersPerSecond * dt
                        + speed.omegaRadiansPerSecond * botRadius * dt;

        // to track shots:
        // 1) look for shooter rpm moving down from the setpoint
        // 2) then look for it to start increasing back towards setpoint
        // 3) count that as a shot
        // if(r.spindexter.)

        Logger.recordOutput("Stats", stats);
    }
}
