package frc.robot.subsystems.shooter;

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

public class ShooterIOHardware implements ShooterIO {

    private final TalonFX wheel;
    private final TalonFX hood;
    private final TalonFX turret;

    private final VoltageOut voltageRequestWheel = new VoltageOut(0);
    private final VoltageOut voltageRequestHood = new VoltageOut(0);
    private final VoltageOut voltageRequestTurret = new VoltageOut(0);

    private final VelocityTorqueCurrentFOC velocityRequestWheel = new VelocityTorqueCurrentFOC(0);
    private final PositionTorqueCurrentFOC positionRequestHood = new PositionTorqueCurrentFOC(0);
    private final PositionTorqueCurrentFOC positionRequestTurret = new PositionTorqueCurrentFOC(0);

    private final StatusSignal<Angle> positionWheel;
    private final StatusSignal<Angle> positionHood;
    private final StatusSignal<Angle> positionTurret;
    private final StatusSignal<Voltage> voltageWheel;
    private final StatusSignal<Voltage> voltageHood;
    private final StatusSignal<Voltage> voltageTurret;
    private final StatusSignal<Current> currentWheel;
    private final StatusSignal<Current> currentHood;
    private final StatusSignal<Current> currentTurret;
    private final StatusSignal<Temperature> tempWheel;
    private final StatusSignal<Temperature> tempHood;
    private final StatusSignal<Temperature> tempTurret;
    private final StatusSignal<AngularVelocity> angularVelocityWheel;
    private final StatusSignal<AngularVelocity> angularVelocityHood;
    private final StatusSignal<AngularVelocity> angularVelocityTurret;

    private final Debouncer wheelConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);
    private final Debouncer hoodConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);
    private final Debouncer turretConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);

    public ShooterIOHardware() {
        wheel = new TalonFX(0); // TODO: add motorIDs & CANbus names
        hood = new TalonFX(0); // TODO: add motorIDs & CANbus names
        turret = new TalonFX(0); // TODO: add motorIDs & CANbus names
        // TODO: do motor config

        positionWheel = wheel.getPosition();
        positionHood = hood.getPosition();
        positionTurret = turret.getPosition();
        voltageWheel = wheel.getMotorVoltage();
        voltageHood = hood.getMotorVoltage();
        voltageTurret = turret.getMotorVoltage();
        currentWheel = wheel.getStatorCurrent();
        currentHood = hood.getStatorCurrent();
        currentTurret = turret.getStatorCurrent();
        tempWheel = wheel.getDeviceTemp();
        tempHood = hood.getDeviceTemp();
        tempTurret = turret.getDeviceTemp();
        angularVelocityWheel = wheel.getVelocity();
        angularVelocityHood = hood.getVelocity();
        angularVelocityTurret = turret.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                positionWheel,
                positionHood,
                positionTurret,
                voltageWheel,
                voltageHood,
                voltageTurret,
                currentWheel,
                currentHood,
                currentTurret,
                tempWheel,
                tempHood,
                tempTurret,
                angularVelocityWheel,
                angularVelocityHood,
                angularVelocityTurret);
        ParentDevice.optimizeBusUtilizationForAll(wheel, hood, turret);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        StatusCode wheelStatus =
                BaseStatusSignal.refreshAll(
                        positionWheel, voltageWheel, currentWheel, tempWheel, angularVelocityWheel);
        StatusCode hoodStatus =
                BaseStatusSignal.refreshAll(
                        positionHood, voltageHood, currentHood, tempHood, angularVelocityHood);
        StatusCode turretStatus =
                BaseStatusSignal.refreshAll(
                        positionTurret,
                        voltageTurret,
                        currentTurret,
                        tempTurret,
                        angularVelocityTurret);

        inputs.wheelConnected = wheelConnectedDebounce.calculate(wheelStatus.isOK());
        inputs.hoodConnected = hoodConnectedDebounce.calculate(hoodStatus.isOK());
        inputs.turretConnected = turretConnectedDebounce.calculate(turretStatus.isOK());
        inputs.wheelPosition = positionWheel.getValueAsDouble();
        inputs.hoodPosition = positionHood.getValueAsDouble();
        inputs.turretPosition = positionTurret.getValueAsDouble();
        inputs.wheelVoltage = voltageWheel.getValueAsDouble();
        inputs.hoodVoltage = voltageHood.getValueAsDouble();
        inputs.turretVoltage = voltageTurret.getValueAsDouble();
        inputs.wheelCurrent = currentWheel.getValueAsDouble();
        inputs.hoodCurrent = currentHood.getValueAsDouble();
        inputs.turretCurrent = currentTurret.getValueAsDouble();
        inputs.wheelTemp = tempWheel.getValueAsDouble();
        inputs.hoodTemp = tempHood.getValueAsDouble();
        inputs.turretTemp = tempTurret.getValueAsDouble();
        inputs.wheelVelocity = angularVelocityWheel.getValueAsDouble();
        inputs.hoodVelocity = angularVelocityHood.getValueAsDouble();
        inputs.turretVelocity = angularVelocityTurret.getValueAsDouble();
    }

    @Override
    public void wheelPower(double power) {
        wheel.setControl(voltageRequestWheel.withOutput(power * 12));
    }

    @Override
    public void hoodPower(double power) {
        hood.setControl(voltageRequestHood.withOutput(power * 12));
    }

    @Override
    public void turretPower(double power) {
        turret.setControl(voltageRequestTurret.withOutput(power * 12));
    }

    @Override
    public void setTurretAngle(double turretAngle) {
        turret.setControl(positionRequestTurret.withPosition(turretAngle));
    }

    @Override
    public void setHoodAngle(double hoodAngle) {
        hood.setControl(positionRequestHood.withPosition(hoodAngle));
    }

    @Override
    public void setSpeed(double speed) {
        wheel.setControl(velocityRequestWheel.withVelocity(speed));
    }
}
