package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.spindexter.SpindexterIOSim;
import java.util.HashMap;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

public class ShooterIOSim implements ShooterIO {
    private final IntakeIOSim iis;
    private final SwerveDriveSimulation swerveSim;
    private final SpindexterIOSim spinSim;
    private Shooter shooter;

    private final FlywheelSim wheel;
    private final SingleJointedArmSim hood;
    private final SingleJointedArmSim turret;

    private final PIDController wheelController;
    private final PIDController hoodController;
    private final PIDController turretController;

    private boolean wheelClosedLoop = false;
    static final boolean dontModelPID = true;

    private double wheelControlVoltage = 0;
    private double hoodControlVoltage = 0;
    private double turretControlVoltage = 0;
    private double wheelFeedfwdVoltage = 0;
    private double hoodFeedfwdVoltage = 0;
    private double turretFeedfwdVoltage = 0;
    private final double turretKF =
            1.0 / Units.radiansToDegrees(DCMotor.getKrakenX60Foc(1).KvRadPerSecPerVolt) * 10;
    private final double wheelKF =
            1.0
                    / Units.radiansPerSecondToRotationsPerMinute(
                            DCMotor.getKrakenX60Foc(1).KvRadPerSecPerVolt)
                    / 1.5;
    private boolean hoodClosedLoop = false;
    private boolean turretClosedLoop = false;

    // 10 balls per second
    private final double shotTime = 1 / 5.0;
    private final Timer shotClock = new Timer();

    public static class MapData {
        double launchTime;
        double predictedFlightTime;
    }

    public HashMap<RebuiltFuelOnFly, MapData> timerMap = new HashMap<>();
    public int failedLookups = 0;

    public ShooterIOSim(IntakeIOSim iis, SwerveDriveSimulation swerveSim, SpindexterIOSim spinSim) {
        this.iis = iis;
        this.swerveSim = swerveSim;
        this.spinSim = spinSim;

        wheel =
                new FlywheelSim(
                        LinearSystemId.createFlywheelSystem(
                                DCMotor.getKrakenX60Foc(1), 0.006, 1.0 / 1.5),
                        DCMotor.getKrakenX60Foc(1));

        wheelController = new PIDController(0.10, 0, 0.0001); // V/rpm
        hoodController = new PIDController(0.1, 0, 0.0); // V/deg
        turretController = new PIDController(0.35, 0, 0.05); // V/deg

        hood =
                new SingleJointedArmSim(
                        DCMotor.getKrakenX60Foc(1),
                        10,
                        0.05,
                        Units.inchesToMeters(2),
                        Units.degreesToRadians(0),
                        Units.degreesToRadians(90),
                        false,
                        Units.degreesToRadians(45));

        turret =
                new SingleJointedArmSim(
                        DCMotor.getKrakenX60(1),
                        10,
                        0.2,
                        Units.inchesToMeters(4),
                        Units.degreesToRadians(0),
                        Units.degreesToRadians(Constants.maximumTurretAngle),
                        false,
                        0);

        shotClock.start();
    }

    public void registerShooter(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        if (!dontModelPID) {
            if (wheelClosedLoop) {
                wheelControlVoltage =
                        wheelFeedfwdVoltage
                                + wheelController.calculate(wheel.getAngularVelocityRPM());
            } else {
                wheelController.reset();
            }
            if (hoodClosedLoop) {
                hoodControlVoltage =
                        hoodController.calculate(Units.radiansToDegrees(hood.getAngleRads()));

            } else {
                hoodController.reset();
            }
            if (turretClosedLoop) {
                turretControlVoltage =
                        turretFeedfwdVoltage
                                + turretController.calculate(
                                        Units.radiansToDegrees(turret.getAngleRads()));

            } else {
                turretController.reset();
            }

            hoodControlVoltage = MathUtil.clamp(hoodControlVoltage, -12, 12);
            wheelControlVoltage = MathUtil.clamp(wheelControlVoltage, -12, 12);
            turretControlVoltage = MathUtil.clamp(turretControlVoltage, -12, 12);
            if (DriverStation.isDisabled()) {
                hoodControlVoltage = 0;
                wheelControlVoltage = 0;
                turretControlVoltage = 0;
            }

            hood.setInputVoltage(hoodControlVoltage);
            wheel.setInputVoltage(wheelControlVoltage);
            turret.setInputVoltage(turretControlVoltage);
        }

        // if indexer is feeding a ball to the shooter
        if (spinSim.areWeRunning && shotClock.hasElapsed(shotTime)) {
            if (iis.hasBall()) {
                shotClock.reset();

                // pull energy out of the wheel
                // do this first as we shoot with the low velocity
                wheel.setAngularVelocity(
                        wheel.getAngularVelocityRadPerSec()
                                - Units.rotationsPerMinuteToRadiansPerSecond(50));

                Rotation2d turretAngle = Rotation2d.fromRadians(turret.getAngleRads());
                RebuiltFuelOnFly fuel =
                        new RebuiltFuelOnFly(
                                swerveSim.getSimulatedDriveTrainPose().getTranslation(),
                                Constants.shooterLocOnBot.rotateBy(turretAngle.unaryMinus()),
                                swerveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                turretAngle.plus(
                                        swerveSim.getSimulatedDriveTrainPose().getRotation()),
                                Inches.of(18),
                                MetersPerSecond.of(
                                        wheel.getAngularVelocityRPM()
                                                / 60
                                                * Math.PI
                                                * Units.inchesToMeters(1.375)),
                                Radians.of(hood.getAngleRads()));
                fuel.enableBecomesGamePieceOnFieldAfterTouchGround();
                fuel.withTargetPosition(
                        () ->
                                new Translation3d(
                                        FieldConstants.Hub.center.getX(),
                                        FieldConstants.Hub.center.getY(),
                                        Units.inchesToMeters(72)));
                fuel.withTargetTolerance(
                        new Translation3d(
                                Units.inchesToMeters(41.7 / 2),
                                Units.inchesToMeters(41.7 / 2),
                                0.1));
                fuel.withProjectileTrajectoryDisplayCallBack(
                        (pose3d) ->
                                Logger.recordOutput(
                                        "Sim/SuccessfulShot", pose3d.toArray(Pose3d[]::new)),
                        (pose3d) ->
                                Logger.recordOutput(
                                        "Sim/MissedShot", pose3d.toArray(Pose3d[]::new)));

                SimulatedArena.getInstance().addGamePieceProjectile(fuel);
                // uncomment if we want to measure sim flight time again
                // if (fuel.willHitTarget()) {
                //     MapData data = new MapData();
                //     data.launchTime = Timer.getFPGATimestamp();
                //     data.predictedFlightTime = shooter.getLastFlightTime();
                //     timerMap.put(fuel, data);

                //     fuel.setHitTargetCallBack(
                //             new Runnable() {
                //                 private RebuiltFuelOnFly selfRef = fuel;

                //                 public void run() {
                //                     if (selfRef != null && timerMap.containsKey(selfRef)) {
                //                         double time = Timer.getFPGATimestamp();
                //                         MapData d = timerMap.get(selfRef);
                //                         double flightTime = time - d.launchTime;
                //                         Logger.recordOutput(
                //                                 "FieldSimultaion/FlightTime", flightTime);
                //                         Logger.recordOutput(
                //                                 "FieldSimultaion/FlightTimeError",
                //                                 flightTime - d.predictedFlightTime);
                //                         timerMap.remove(selfRef);
                //                         selfRef = null;
                //                     } else {
                //                         Logger.recordOutput(
                //                                 "FieldSimulation/FailedLookups",
                // ++failedLookups);
                //                     }
                //                 }
                //             });
                // }
            }
        }

        if (!dontModelPID) {
            hood.update(0.02);
            wheel.update(0.02);
            turret.update(0.02);
        }

        inputs.hoodConnected = true;
        inputs.hoodPosition = Units.radiansToDegrees(hood.getAngleRads());
        inputs.hoodVelocity = Units.radiansToDegrees(hood.getVelocityRadPerSec());
        inputs.hoodVoltage = hoodControlVoltage;
        inputs.hoodCurrent = hood.getCurrentDrawAmps();
        inputs.hoodTemp = 0;

        inputs.turretConnected = true;
        inputs.turretPosition = Units.radiansToDegrees(turret.getAngleRads());
        inputs.turretVelocity = Units.radiansToDegrees(turret.getVelocityRadPerSec());
        inputs.turretVoltage = turretControlVoltage;
        inputs.turretCurrent = turret.getCurrentDrawAmps();
        inputs.turretTemp = 0;

        inputs.wheelConnected = true;
        inputs.wheelVelocity = wheel.getAngularVelocityRPM();
        inputs.wheelPosition +=
                0.02
                        * Units.radiansToDegrees(
                                Units.rotationsPerMinuteToRadiansPerSecond(inputs.wheelVelocity));
        inputs.wheelVoltage = wheelControlVoltage;
        inputs.wheelCurrent = wheel.getCurrentDrawAmps();
        inputs.wheelTemp = 0;
    }

    public void setTurretAngle(double turretAngle, double velocity) {
        if (dontModelPID) {
            turret.setState(Math.toRadians(turretAngle), Math.toRadians(velocity));
        } else {
            turretClosedLoop = true;
            turretFeedfwdVoltage = turretKF * velocity;
            turretController.setSetpoint(turretAngle);
        }
    }

    @Override
    public void setHoodAngle(double hoodAngle) {
        if (dontModelPID) {
            hood.setState(Math.toRadians(hoodAngle), 0);
        } else {
            hoodClosedLoop = true;
            hoodController.setSetpoint(hoodAngle);
        }
    }

    @Override
    public void setSpeed(double speed) {
        if (dontModelPID) {
            wheel.setAngularVelocity(Units.rotationsPerMinuteToRadiansPerSecond(speed));
        } else {
            wheelClosedLoop = true;
            wheelFeedfwdVoltage = wheelKF * speed;
            wheelController.setSetpoint(speed);
        }
    }

    @Override
    public void wheelPower(double power) {
        wheelClosedLoop = false;
        wheelControlVoltage = power * 12;
    }

    @Override
    public void hoodPower(double power) {
        hoodClosedLoop = false;
        hoodControlVoltage = power * 12;
    }

    @Override
    public void turretPower(double power) {
        turretClosedLoop = false;
        turretControlVoltage = power * 12;
    }
}
