package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
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

public class ClimberIOHardware implements ClimberIO {

    private final TalonFX climb1;
    private final TalonFX climb2;

    private final VoltageOut voltageRequest1 = new VoltageOut(0);
    private final VoltageOut voltageRequest2 = new VoltageOut(0);
    private final PositionTorqueCurrentFOC positionRequest1 = new PositionTorqueCurrentFOC(0);
    private final PositionTorqueCurrentFOC positionRequest2 = new PositionTorqueCurrentFOC(0);

    private final StatusSignal<Angle> positionClimb1;
    private final StatusSignal<Angle> positionClimb2;
    private final StatusSignal<Voltage> voltageClimb1;
    private final StatusSignal<Voltage> voltageClimb2;
    private final StatusSignal<Current> currentClimb1;
    private final StatusSignal<Current> currentClimb2;
    private final StatusSignal<Temperature> tempClimb1;
    private final StatusSignal<Temperature> tempClimb2;
    private final StatusSignal<AngularVelocity> angularVelocityClimb1;
    private final StatusSignal<AngularVelocity> angularVelocityClimb2;

    private final Debouncer climb1ConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);
    private final Debouncer climb2ConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);

    public ClimberIOHardware() {
        climb1 = new TalonFX(0); // TODO: add motorIDs & CANbus names
        climb2 = new TalonFX(0); // TODO: add motorIDs & CANbus names
        // TODO: do motor config

        positionClimb1 = climb1.getPosition();
        positionClimb2 = climb2.getPosition();
        voltageClimb1 = climb1.getMotorVoltage();
        voltageClimb2 = climb2.getMotorVoltage();
        currentClimb1 = climb1.getStatorCurrent();
        currentClimb2 = climb2.getStatorCurrent();
        tempClimb1 = climb1.getDeviceTemp();
        tempClimb2 = climb2.getDeviceTemp();
        angularVelocityClimb1 = climb1.getVelocity();
        angularVelocityClimb2 = climb2.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                positionClimb1,
                positionClimb2,
                voltageClimb1,
                voltageClimb2,
                currentClimb1,
                currentClimb2,
                tempClimb1,
                tempClimb2,
                angularVelocityClimb1,
                angularVelocityClimb2);
        ParentDevice.optimizeBusUtilizationForAll(climb1, climb2);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        StatusCode climb1Status =
                BaseStatusSignal.refreshAll(
                        positionClimb1,
                        voltageClimb1,
                        currentClimb1,
                        tempClimb1,
                        angularVelocityClimb1);
        StatusCode climb2Status =
                BaseStatusSignal.refreshAll(
                        positionClimb2,
                        voltageClimb2,
                        currentClimb2,
                        tempClimb2,
                        angularVelocityClimb2);
        inputs.climb1Connected = climb1ConnectedDebounce.calculate(climb1Status.isOK());
        inputs.climb2Connected = climb2ConnectedDebounce.calculate(climb2Status.isOK());
        inputs.climb1Position = positionClimb1.getValueAsDouble();
        inputs.climb2Position = positionClimb2.getValueAsDouble();
        inputs.climb1Voltage = voltageClimb1.getValueAsDouble();
        inputs.climb2Voltage = voltageClimb2.getValueAsDouble();
        inputs.climb1Current = currentClimb1.getValueAsDouble();
        inputs.climb2Current = currentClimb2.getValueAsDouble();
        inputs.climb1Temp = tempClimb1.getValueAsDouble();
        inputs.climb2Temp = tempClimb2.getValueAsDouble();
        inputs.climb1Velocity = angularVelocityClimb1.getValueAsDouble();
        inputs.climb2Velocity = angularVelocityClimb2.getValueAsDouble();
    }

    @Override
    public void setPower1(double power) {
        climb1.setControl(voltageRequest1.withOutput(power * 12));
    }

    @Override
    public void setPower2(double power) {
        climb2.setControl(voltageRequest2.withOutput(power * 12));
    }

    @Override
    public void setPos1(double position) {
        climb1.setControl(positionRequest1.withPosition(position));
    }

    @Override
    public void setPos2(double position) {
        climb2.setControl(positionRequest2.withPosition(position));
    }
}
