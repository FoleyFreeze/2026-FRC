package frc.robot.subsystems.spindexter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class SpindexterIOHardware implements SpindexterIO {

    private final TalonFX spin;

    private final VoltageOut voltageRequestSpin = new VoltageOut(0);
    private final VelocityTorqueCurrentFOC velocityRequestSpin = new VelocityTorqueCurrentFOC(0);

    private final StatusSignal<Angle> positionSpin;
    private final StatusSignal<Voltage> voltageSpin;
    private final StatusSignal<Current> currentSpin;
    private final StatusSignal<Temperature> tempSpin;
    private final StatusSignal<AngularVelocity> angularVelocitySpin;

    private final Debouncer spinConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);

    public SpindexterIOHardware() {
        spin = new TalonFX(0); // TODO: add motorIDs & CANbus names
        // TODO: do motor config

        positionSpin = spin.getPosition();
        voltageSpin = spin.getMotorVoltage();
        currentSpin = spin.getStatorCurrent();
        tempSpin = spin.getDeviceTemp();
        angularVelocitySpin = spin.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50, positionSpin, voltageSpin, currentSpin, tempSpin, angularVelocitySpin);
        ParentDevice.optimizeBusUtilizationForAll(spin);
    }

    @Override
    public void updateInputs(SpindexterIOInputs inputs) {
        StatusCode spinStatus =
                BaseStatusSignal.refreshAll(
                        positionSpin, voltageSpin, currentSpin, tempSpin, angularVelocitySpin);
        inputs.spinConnected = spinConnectedDebounce.calculate(spinStatus.isOK());
        inputs.spinPosition = positionSpin.getValueAsDouble();
        inputs.spinVoltage = voltageSpin.getValueAsDouble();
        inputs.spinCurrent = currentSpin.getValueAsDouble();
        inputs.spinTemp = tempSpin.getValueAsDouble();
        inputs.spinVelocity = angularVelocitySpin.getValueAsDouble();
    }

    @Override
    public void spinPower(double power) {
        spin.setControl(voltageRequestSpin.withOutput(power * 12));
    }

    @Override
    public void spinSpeed(double speed) {
        spin.setControl(velocityRequestSpin.withAcceleration(speed));
    }
}
