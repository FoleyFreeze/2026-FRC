package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
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

public class IntakeIOHardware implements IntakeIO {

    private final TalonFX wheel;
    private final TalonFX arm;

    private final VoltageOut voltageRequestWheel = new VoltageOut(0);
    private final VoltageOut voltageRequestArm = new VoltageOut(0);

    private final VelocityTorqueCurrentFOC velocityRequestWheel = new VelocityTorqueCurrentFOC(0);
    private final PositionTorqueCurrentFOC positionRequestArm = new PositionTorqueCurrentFOC(0);

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
        wheel = new TalonFX(0); // TODO: add motorIDs & CANbus names
        arm = new TalonFX(0); // TODO: add motorIDs & CANbus names
        // TODO: do motor config

        positionWheel = wheel.getPosition();
        positionArm = arm.getPosition();
        voltageWheel = wheel.getMotorVoltage();
        voltageArm = arm.getMotorVoltage();
        currentWheel = wheel.getStatorCurrent();
        currentArm = arm.getStatorCurrent();
        tempWheel = wheel.getDeviceTemp();
        tempArm = arm.getDeviceTemp();
        angularVelocityWheel = wheel.getVelocity();
        angularVelocityArm = arm.getVelocity();

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
        ParentDevice.optimizeBusUtilizationForAll(wheel, arm);
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
        inputs.wheelPosition = positionWheel.getValueAsDouble();
        inputs.armPosition = positionArm.getValueAsDouble();
        inputs.wheelVoltage = voltageWheel.getValueAsDouble();
        inputs.armVoltage = voltageArm.getValueAsDouble();
        inputs.wheelCurrent = currentWheel.getValueAsDouble();
        inputs.armCurrent = currentArm.getValueAsDouble();
        inputs.wheelTemp = tempWheel.getValueAsDouble();
        inputs.armTemp = tempArm.getValueAsDouble();
        inputs.wheelVelocity = angularVelocityWheel.getValue().in(RPM);
        inputs.armVelocity = angularVelocityArm.getValueAsDouble();
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
        arm.setControl(voltageRequestArm.withOutput(power * 12));
    }

    @Override
    public void armAngle(double angle) {
        arm.setControl(positionRequestArm.withPosition(angle));
    }
}
