package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOHardware implements IntakeIO {

    private final TalonFX wheel;
    private final TalonFX intakeBar;

    private final VoltageOut voltageRequestWheel = new VoltageOut(0);
    private final VoltageOut voltageRequestArm = new VoltageOut(0);

    private final VelocityTorqueCurrentFOC velocityRequestWheel = new VelocityTorqueCurrentFOC(0);
    private final PositionTorqueCurrentFOC positionRequestArm =
            new PositionTorqueCurrentFOC(0).withSlot(0);
    private final MotionMagicTorqueCurrentFOC motionRequestArm =
            new MotionMagicTorqueCurrentFOC(0).withSlot(1);

    private final StatusSignal<Angle> positionWheel;
    private final StatusSignal<Angle> positionArm;
    private final StatusSignal<Voltage> voltageWheel;
    private final StatusSignal<Voltage> voltageArm;
    private final StatusSignal<Current> currentWheel;
    private final StatusSignal<Current> currentArm;
    private final StatusSignal<Temperature> tempWheel;
    private final StatusSignal<Temperature> tempArm;
    private final StatusSignal<AngularVelocity> angularVelocityWheel;
    private final StatusSignal<AngularVelocity> angularVelocityArm;

    private final Debouncer wheelConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);
    private final Debouncer armConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);

    public IntakeIOHardware() {
        var cfg = new TalonFXConfiguration();
        wheel = new TalonFX(3);


        intakeBar = new TalonFX(2);
        cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        cfg.TorqueCurrent.PeakReverseTorqueCurrent = -25;
        //TODO: not done with intake arm, left off after torque current, check slack for info



        positionWheel = wheel.getPosition();
        positionArm = intakeBar.getPosition();
        voltageWheel = wheel.getMotorVoltage();
        voltageArm = intakeBar.getMotorVoltage();
        currentWheel = wheel.getStatorCurrent();
        currentArm = intakeBar.getStatorCurrent();
        tempWheel = wheel.getDeviceTemp();
        tempArm = intakeBar.getDeviceTemp();
        angularVelocityWheel = wheel.getVelocity();
        angularVelocityArm = intakeBar.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                positionWheel,
                positionArm,
                voltageWheel,
                voltageArm,
                currentWheel,
                currentArm,
                tempWheel,
                tempArm,
                angularVelocityWheel,
                angularVelocityArm);
        ParentDevice.optimizeBusUtilizationForAll(wheel, intakeBar);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        StatusCode wheelStatus =
                BaseStatusSignal.refreshAll(
                        positionWheel, voltageWheel, currentWheel, tempWheel, angularVelocityWheel);
        StatusCode armStatus =
                BaseStatusSignal.refreshAll(
                        positionArm, voltageArm, currentArm, tempArm, angularVelocityArm);
        inputs.wheelConnected = wheelConnectedDebounce.calculate(wheelStatus.isOK());
        inputs.armConnected = armConnectedDebounce.calculate(armStatus.isOK());
        inputs.wheelPosition = positionWheel.getValue().in(Rotations);
        inputs.armPosition = positionArm.getValue().in(Rotations);
        inputs.wheelVoltage = voltageWheel.getValueAsDouble();
        inputs.armVoltage = voltageArm.getValueAsDouble();
        inputs.wheelCurrent = currentWheel.getValueAsDouble();
        inputs.armCurrent = currentArm.getValueAsDouble();
        inputs.wheelTemp = tempWheel.getValueAsDouble();
        inputs.armTemp = tempArm.getValueAsDouble();
        inputs.wheelVelocity = angularVelocityWheel.getValue().in(RPM);
        inputs.armVelocity = angularVelocityArm.getValue().in(RPM);
    }

    @Override
    public void wheelPower(double power) {
        wheel.setControl(voltageRequestWheel.withOutput(power * 12));
    }

    @Override
    public void wheelSpeed(double speed) {
        wheel.setControl(velocityRequestWheel.withVelocity(speed));
    }

    @Override
    public void armPower(double power) {
        intakeBar.setControl(voltageRequestArm.withOutput(power * 12));
    }

    @Override
    public void armAngle(double rotations) {
        intakeBar.setControl(positionRequestArm.withPosition(rotations));
    }

    @Override
    public void armMotion(double rotations) {
        intakeBar.setControl(motionRequestArm.withPosition(rotations));
    }
}
