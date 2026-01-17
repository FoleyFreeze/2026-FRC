package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.intake.IntakeIOSim;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

public class ShooterIOSim implements ShooterIO {
    private final IntakeIOSim iis;

    private final FlywheelSim wheel;
    private final SingleJointedArmSim hood;
    private final SingleJointedArmSim turret;

    private final PIDController wheelController;
    private final PIDController hoodController;
    private final PIDController turretController;

    private boolean wheelClosedLoop = false;

    private double wheelControlVoltage = 0;
    private double hoodControlVoltage = 0;
    private double turretControlVoltage = 0;
    private double wheelFeedfwdVoltage = 0;
    private double hoodFeedfwdVoltage = 0;
    private double turretFeedfwdVoltage = 0;
    private final double wheelKF = 1000 / 12.0; // 1000rpm at 12v?
    private boolean hoodClosedLoop = false;
    private boolean turretClosedLoop = false;

    public ShooterIOSim(IntakeIOSim iis) {
        this.iis = iis;

        wheel =
                new FlywheelSim(
                        LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.0005, 1),
                        DCMotor.getKrakenX60Foc(1));
        wheelController = new PIDController(2, 0, 0);
        hoodController = new PIDController(0.2, 0, 0);
        turretController = new PIDController(0.2, 0, 0);

        hood =
                new SingleJointedArmSim(
                        DCMotor.getKrakenX60Foc(1),
                        10,
                        0.01,
                        Units.inchesToMeters(2),
                        Units.degreesToRadians(15),
                        Units.degreesToRadians(75),
                        true,
                        Units.degreesToRadians(45));

        turret =
                new SingleJointedArmSim(
                        DCMotor.getKrakenX60(1),
                        10,
                        0.1,
                        Units.inchesToMeters(4),
                        Units.degreesToRadians(0),
                        Units.degreesToRadians(45),
                        false,
                        0);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        if (wheelClosedLoop) {
            wheelControlVoltage =
                    wheelFeedfwdVoltage
                            + wheelController.calculate(wheel.getAngularVelocityRadPerSec());
        } else {
            wheelController.reset();
        }
        if (hoodClosedLoop) {
            hoodControlVoltage = hoodController.calculate(hood.getAngleRads());

        } else {
            hoodController.reset();
        }
        if (turretClosedLoop) {
            turretControlVoltage = turretController.calculate(turret.getAngleRads());

        } else {
            turretController.reset();
        }

        hood.setInputVoltage(hoodControlVoltage);
        wheel.setInputVoltage(wheelControlVoltage);
        turret.setInputVoltage(turretControlVoltage);

        // if indexer is feeding a ball to the shooter
        if (wheel.getAngularVelocityRadPerSec() > 9000) {
            if (iis.hasBall()) {
                RebuiltFuelOnFly fuel =
                        new RebuiltFuelOnFly(
                                new Translation2d(),
                                new Translation2d(),
                                new ChassisSpeeds(),
                                Rotation2d.fromRadians(turret.getAngleRads()),
                                Inches.of(18),
                                MetersPerSecond.of(
                                        wheel.getAngularVelocityRPM() * 60 * Math.PI * 2),
                                Radians.of(hood.getAngleRads()));
            }
        }

        hood.update(0.02);
        wheel.update(0.02);
        turret.update(0.02);

        inputs.hoodConnected = true;
        inputs.hoodPosition = hood.getAngleRads();
        inputs.hoodVelocity = hood.getVelocityRadPerSec();
        inputs.hoodVoltage = hoodControlVoltage;
        inputs.hoodCurrent = hood.getCurrentDrawAmps();
        inputs.hoodTemp = 0;

        inputs.turretConnected = true;
        inputs.turretPosition = turret.getAngleRads();
        inputs.turretVelocity = turret.getVelocityRadPerSec();
        inputs.turretVoltage = turretControlVoltage;
        inputs.turretCurrent = turret.getCurrentDrawAmps();
        inputs.turretTemp = 0;

        inputs.wheelConnected = true;
        inputs.wheelVelocity = wheel.getAngularVelocityRadPerSec();
        inputs.wheelPosition += 0.02 * inputs.wheelVelocity;
        inputs.wheelVoltage = wheelControlVoltage;
        inputs.wheelCurrent = wheel.getCurrentDrawAmps();
        inputs.wheelTemp = 0;
    }

    @Override
    public void setAll(double turretAngle, double hoodAngle, double speed) {
        wheelClosedLoop = true;
        wheelFeedfwdVoltage = wheelKF * speed;
        wheelController.setSetpoint(speed);

        turretClosedLoop = true;
        turretController.setSetpoint(turretAngle);
        hoodClosedLoop = true;
        hoodController.setSetpoint(hoodAngle);
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
