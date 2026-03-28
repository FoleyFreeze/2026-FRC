package frc.robot.subsystems.stats;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ConfigButtons;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter.ShootMode;
import org.littletonrobotics.junction.Logger;

public class StatsSubsystem extends SubsystemBase {
    RobotContainer r;
    Stats stats = new Stats();

    double botRadius = Drive.getModuleTranslations()[0].getNorm();

    double prevtime;
    double prevTurretAngle = 0;

    double[] prevWheelDist = new double[4];

    public StatsSubsystem(RobotContainer r) {
        this.r = r;
        prevtime = Timer.getFPGATimestamp();
    }

    @Override
    public void periodic() {
        double time = Timer.getFPGATimestamp();
        double dt = time - prevtime;
        prevtime = time;

        ChassisSpeeds speed = r.drive.getChassisSpeeds();
        double deltaDist =
                Math.sqrt(
                                        speed.vxMetersPerSecond * speed.vxMetersPerSecond
                                                + speed.vyMetersPerSecond * speed.vyMetersPerSecond)
                                * dt
                        + Math.abs(speed.omegaRadiansPerSecond) * botRadius * dt;
        stats.totalTravelDist += deltaDist;

        stats.totalDistFLwheel += getWheelDistDelta(0);
        stats.totalDistFRwheel += getWheelDistDelta(1);
        stats.totalDistRLwheel += getWheelDistDelta(2);
        stats.totalDistRRwheel += getWheelDistDelta(3);

        if (deltaDist > Units.inchesToMeters(2) * dt) {
            stats.totalTravelTime += dt;
        }

        if (r.shooter.ballShotEdge) {
            stats.totalBallShots++;
        }

        if (r.shooter.shootMode == ShootMode.HUB) {
            stats.totalHubShootTime += dt;
        } else if (r.shooter.shootMode == ShootMode.PASS) {
            stats.totalPassShootTime += dt;
        }

        // stats.totalDistanceClimbed;
        // stats.totalClimbs;

        // stats.totalTurretFlips;
        // stats.totalTurretRotations;
        if (prevTurretAngle != 0) {
            stats.totalTurretRotations +=
                    Math.abs(r.shooter.inputs.turretPositionDeg - prevTurretAngle);
        }
        prevTurretAngle = r.shooter.inputs.turretPositionDeg;

        if (DriverStation.isEnabled()) {
            // stats.totalEnabledBallImages;
            stats.totalEnabledTagImages += r.vision.validImages;

            stats.totalButtonsPressed += ConfigButtons.trackButtons();
            stats.totalControlBoardButtonsPressed += ConfigButtons.trackControlBoardButtons();
        } else {
            // run when disabled so edge detector works
            ConfigButtons.trackButtons();
            ConfigButtons.trackControlBoardButtons();
        }

        Logger.recordOutput("Stats", stats);
    }

    private double getWheelDistDelta(int idx) {
        double dist =
                r.drive.modules[idx].inputs.drivePositionRad
                        * TunerConstants.kWheelRadius.in(Meters);
        double deltaDist = Math.abs(dist - prevWheelDist[idx]);
        prevWheelDist[idx] = dist;
        return deltaDist;
    }

    // dont forget to count your flippy things
    public void flippyThing() {
        stats.totalTurretFlips++;
    }
}
