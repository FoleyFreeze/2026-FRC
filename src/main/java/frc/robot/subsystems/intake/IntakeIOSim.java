package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class IntakeIOSim implements IntakeIO {
    private final IntakeSimulation intakeSimulation;

    private final SingleJointedArmSim arm;
    private final PIDController armController;
    private boolean armClosedLoop = false;
    private double armControlVoltage = 0;

    private final int hopperCapacity = 100;
    private final FlywheelSim wheel;
    private final PIDController wheelController;
    private boolean wheelClosedLoop = false;
    private final double wheelKF = 1000 / 12.0; // 1000rpm at 12v?
    private double wheelControlVoltage = 0;
    private double wheelFeedfwdVoltage = 0;

    public IntakeIOSim(AbstractDriveTrainSimulation driveSim) {
        intakeSimulation =
                IntakeSimulation.OverTheBumperIntake(
                        "Fuel",
                        driveSim,
                        Meters.of(
                                Constants.frameWidth
                                        - Units.inchesToMeters(7)), // intake is 21 in wide
                        Inches.of(10),
                        IntakeSide.FRONT,
                        hopperCapacity);
        intakeSimulation.addGamePiecesToIntake(100);

        arm =
                new SingleJointedArmSim(
                        DCMotor.getKrakenX60Foc(1),
                        10,
                        0.5,
                        Units.inchesToMeters(14),
                        Units.degreesToRadians(0),
                        Units.degreesToRadians(90),
                        true,
                        Units.degreesToRadians(90));
        armController = new PIDController(50, 0, 0);

        wheel =
                new FlywheelSim(
                        LinearSystemId.createFlywheelSystem(
                                DCMotor.getKrakenX60Foc(1).withReduction(2), 0.01, 1 / 5.0),
                        DCMotor.getKrakenX60Foc(1));
        wheelController = new PIDController(2, 0, 0);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // Run closed-loop control
        if (armClosedLoop) {
            armControlVoltage =
                    armController.calculate(Units.radiansToRotations(arm.getAngleRads()));

        } else {
            armController.reset();
        }
        if (wheelClosedLoop) {
            wheelControlVoltage =
                    wheelFeedfwdVoltage
                            + wheelController.calculate(
                                    Units.radiansPerSecondToRotationsPerMinute(
                                            wheel.getAngularVelocityRadPerSec()));
        } else {
            wheelController.reset();
        }

        arm.setInputVoltage(armControlVoltage);
        wheel.setInputVoltage(wheelControlVoltage);

        arm.update(0.02);
        wheel.update(0.02);

        inputs.armConnected = true;
        inputs.armPosition = Units.radiansToRotations(arm.getAngleRads());
        inputs.armVelocity = Units.radiansPerSecondToRotationsPerMinute(arm.getVelocityRadPerSec());
        inputs.armVoltage = armControlVoltage;
        inputs.armCurrent = arm.getCurrentDrawAmps();
        inputs.armTemp = 0;

        inputs.wheelConnected = true;
        inputs.wheelVelocity =
                Units.radiansPerSecondToRotationsPerMinute(wheel.getAngularVelocityRadPerSec());
        inputs.wheelPosition += 0.02 * inputs.wheelVelocity / 60.0;
        inputs.wheelVoltage = wheelControlVoltage;
        inputs.wheelCurrent = wheel.getCurrentDrawAmps();
        inputs.wheelTemp = 0;

        // do sim things
        if (inputs.armPosition < Units.degreesToRadians(10)) {
            intakeSimulation.startIntake();
        } else {
            intakeSimulation.stopIntake();
        }
    }

    public boolean hasBall() {
        return intakeSimulation.obtainGamePieceFromIntake();
    }

    @Override
    public void armPower(double power) {
        armClosedLoop = false;
        armControlVoltage = power * 12;
    }

    @Override
    public void armAngle(double rotations) {
        armClosedLoop = true;
        armController.setSetpoint(-rotations + 0.25);
    }

    @Override
    public void armMotion(double rotations) {
        armAngle(rotations);
    }

    @Override
    public void wheelSpeed(double speed) {
        wheelClosedLoop = true;
        wheelFeedfwdVoltage = wheelKF * speed;
        wheelController.setSetpoint(speed);
    }

    @Override
    public void wheelPower(double power) {
        wheelClosedLoop = false;
        wheelControlVoltage = power * 12;
    }
}
