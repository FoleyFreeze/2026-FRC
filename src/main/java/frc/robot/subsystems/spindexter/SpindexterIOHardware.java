package frc.robot.subsystems.spindexter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;

public class SpindexterIOHardware implements SpindexterIO {

    private final TalonFX spin;
    private final TalonFX gate;

    private final VoltageOut voltageRequestSpin = new VoltageOut(0);
    private final VoltageOut voltageRequestGate = new VoltageOut(0);

    private final MotionMagicVelocityTorqueCurrentFOC velocityRequestSpin =
            new MotionMagicVelocityTorqueCurrentFOC(0);
    private final MotionMagicVelocityTorqueCurrentFOC velocityRequestGate =
            new MotionMagicVelocityTorqueCurrentFOC(0);

    private final StatusSignal<Angle> positionSpin;
    private final StatusSignal<Voltage> voltageSpin;
    private final StatusSignal<Current> currentSpin;
    private final StatusSignal<Temperature> tempSpin;
    private final StatusSignal<AngularVelocity> angularVelocitySpin;

    private final StatusSignal<Angle> positionGate;
    private final StatusSignal<Voltage> voltageGate;
    private final StatusSignal<Current> currentGate;
    private final StatusSignal<Temperature> tempGate;
    private final StatusSignal<AngularVelocity> angularVelocityGate;

    private final Debouncer spinConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);
    private final Debouncer gateConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);

    public SpindexterIOHardware() {

        var cfg = new TalonFXConfiguration();
        spin = new TalonFX(6, TunerConstants.kCANBus);
        cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.Slot0.kP = 11;
        cfg.Slot0.kS = 10;
        cfg.Slot0.kV = 0.17;
        cfg.TorqueCurrent.PeakForwardTorqueCurrent = 100;
        cfg.TorqueCurrent.PeakReverseTorqueCurrent = -100;
        cfg.MotionMagic.MotionMagicAcceleration = 250;
        cfg.MotionMagic.MotionMagicJerk = 1000;
        spin.getConfigurator().apply(cfg);

        gate = new TalonFX(17, TunerConstants.kCANBus);
        cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.Slot0.kP = 14;
        cfg.Slot0.kS = 10;
        cfg.Slot0.kV = 0.17;
        cfg.TorqueCurrent.PeakForwardTorqueCurrent = 100;
        cfg.TorqueCurrent.PeakReverseTorqueCurrent = -100;
        cfg.MotionMagic.MotionMagicAcceleration = 180;
        cfg.MotionMagic.MotionMagicJerk = 1000;
        gate.getConfigurator().apply(cfg);

        positionSpin = spin.getPosition();
        voltageSpin = spin.getMotorVoltage();
        currentSpin = spin.getStatorCurrent();
        tempSpin = spin.getDeviceTemp();
        angularVelocitySpin = spin.getVelocity();

        positionGate = gate.getPosition();
        voltageGate = gate.getMotorVoltage();
        currentGate = gate.getStatorCurrent();
        tempGate = gate.getDeviceTemp();
        angularVelocityGate = gate.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50, positionSpin, voltageSpin, currentSpin, tempSpin, angularVelocitySpin);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50, positionGate, voltageGate, currentGate, tempGate, angularVelocityGate);
        // TODO: temporary
        BaseStatusSignal.setUpdateFrequencyForAll(200, currentGate, currentSpin);

        ParentDevice.optimizeBusUtilizationForAll(spin);
        ParentDevice.optimizeBusUtilizationForAll(gate);
    }

    @Override
    public void updateInputs(SpindexterIOInputs inputs) {
        StatusCode spinStatus =
                BaseStatusSignal.refreshAll(
                        positionSpin, voltageSpin, currentSpin, tempSpin, angularVelocitySpin);
        StatusCode gateStatus =
                BaseStatusSignal.refreshAll(
                        positionGate, voltageGate, currentGate, tempGate, angularVelocityGate);

        inputs.spinConnected = spinConnectedDebounce.calculate(spinStatus.isOK());
        inputs.spinPosition = positionSpin.getValue().in(Rotations);
        inputs.spinVoltage = voltageSpin.getValueAsDouble();
        inputs.spinCurrent = currentSpin.getValueAsDouble();
        inputs.spinTemp = tempSpin.getValueAsDouble();
        inputs.spinVelocity = angularVelocitySpin.getValue().in(RPM);

        inputs.gateConnected = gateConnectedDebounce.calculate(gateStatus.isOK());
        inputs.gatePosition = positionGate.getValue().in(Rotations);
        inputs.gateVoltage = voltageGate.getValueAsDouble();
        inputs.gateCurrent = currentGate.getValueAsDouble();
        inputs.gateTemp = tempGate.getValueAsDouble();
        inputs.gateVelocity = angularVelocityGate.getValue().in(RPM);
    }

    @Override
    public void spinPower(double power) {
        spin.setControl(voltageRequestSpin.withOutput(power * 12));
    }

    @Override
    public void gatePower(double power) {
        gate.setControl(voltageRequestGate.withOutput(power * 12));
    }

    @Override
    public void spinSpeed(double speed) {
        spin.setControl(velocityRequestSpin.withVelocity(speed / 60));
    }

    @Override
    public void gateSpeed(double speed) {
        gate.setControl(velocityRequestGate.withVelocity(speed / 60));
    }
}
