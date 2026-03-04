package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOHardware implements ShooterIO {

    private static final boolean hasTurret = false;

    private final TalonFX wheel;
    private final TalonFX wheel2;
    private final TalonFX hood;
    private TalonFX turret;

    private final VoltageOut voltageRequestWheel = new VoltageOut(0);
    private final VoltageOut voltageRequestHood = new VoltageOut(0);
    private VoltageOut voltageRequestTurret = new VoltageOut(0);

    private final VelocityTorqueCurrentFOC velocityRequestWheel = new VelocityTorqueCurrentFOC(0);
    private final PositionTorqueCurrentFOC positionRequestHood = new PositionTorqueCurrentFOC(0);
    private PositionTorqueCurrentFOC positionRequestTurret = new PositionTorqueCurrentFOC(0);

    private final StatusSignal<Angle> positionWheel;
    private final StatusSignal<Angle> positionHood;
    private StatusSignal<Angle> positionTurret;
    private final StatusSignal<Voltage> voltageWheel;
    private final StatusSignal<Voltage> voltageHood;
    private StatusSignal<Voltage> voltageTurret;
    private final StatusSignal<Current> currentWheel;
    private final StatusSignal<Current> currentHood;
    private StatusSignal<Current> currentTurret;
    private final StatusSignal<Temperature> tempWheel;
    private final StatusSignal<Temperature> tempHood;
    private StatusSignal<Temperature> tempTurret;
    private final StatusSignal<AngularVelocity> angularVelocityWheel;
    private final StatusSignal<AngularVelocity> angularVelocityHood;
    private StatusSignal<AngularVelocity> angularVelocityTurret;

    private final Debouncer wheelConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);
    private final Debouncer hoodConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);
    private final Debouncer turretConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);

    public ShooterIOHardware() {
        wheel = new TalonFX(12); 

        var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.Slot0.kP = 10;
        cfg.Slot0.kS =5;
        cfg.Slot0.kV = 0.17;
        cfg.TorqueCurrent.PeakForwardTorqueCurrent = 100;
        cfg.TorqueCurrent.PeakReverseTorqueCurrent = -100;
        wheel.getConfigurator().apply(cfg);

        wheel2 = new TalonFX(13);
        cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        wheel2.getConfigurator().apply(cfg);
        wheel2.setControl(new Follower(12, MotorAlignmentValue.Opposed));


        hood = new TalonFX(15);
        cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.Slot0.kP = 1800;
        cfg.Slot0.kD = 80;
        cfg.TorqueCurrent.PeakForwardTorqueCurrent = 25;
        cfg.TorqueCurrent.PeakReverseTorqueCurrent = -25;
        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        cfg.Feedback.FeedbackRemoteSensorID = 15;
        cfg.Feedback.RotorToSensorRatio = 9.0 * 48.0 / 20.0;
        cfg.Feedback.SensorToMechanismRatio = 20.0 / 48.0 * 314.0 / 20.0;
        hood.getConfigurator().apply(cfg);


        if (hasTurret) {
            turret = new TalonFX(0); 
            voltageTurret = turret.getMotorVoltage();
            currentTurret = turret.getStatorCurrent();
            tempTurret = turret.getDeviceTemp();
            angularVelocityTurret = turret.getVelocity();
        }

        positionWheel = wheel.getPosition();
        positionHood = hood.getPosition();
        voltageWheel = wheel.getMotorVoltage();
        voltageHood = hood.getMotorVoltage();
        currentWheel = wheel.getStatorCurrent();
        currentHood = hood.getStatorCurrent();
        tempWheel = wheel.getDeviceTemp();
        tempHood = hood.getDeviceTemp();
        angularVelocityWheel = wheel.getVelocity();
        angularVelocityHood = hood.getVelocity();

        if (hasTurret) {
            BaseStatusSignal.setUpdateFrequencyForAll(
                    50,
                    positionTurret,
                    voltageTurret,
                    currentTurret,
                    tempTurret,
                    angularVelocityTurret);
            ParentDevice.optimizeBusUtilizationForAll(turret);
        }

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                positionWheel,
                positionHood,
                voltageWheel,
                voltageHood,
                currentWheel,
                currentHood,
                tempWheel,
                tempHood,
                angularVelocityWheel,
                angularVelocityHood);
        ParentDevice.optimizeBusUtilizationForAll(wheel, hood);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        StatusCode wheelStatus =
                BaseStatusSignal.refreshAll(
                        positionWheel, voltageWheel, currentWheel, tempWheel, angularVelocityWheel);
        StatusCode hoodStatus =
                BaseStatusSignal.refreshAll(
                        positionHood, voltageHood, currentHood, tempHood, angularVelocityHood);

        if (hasTurret) {
            StatusCode turretStatus =
                    BaseStatusSignal.refreshAll(
                            positionTurret,
                            voltageTurret,
                            currentTurret,
                            tempTurret,
                            angularVelocityTurret);
            inputs.turretConnected = turretConnectedDebounce.calculate(turretStatus.isOK());
            inputs.turretPosition = positionTurret.getValue().in(Degrees);
            inputs.turretVoltage = voltageTurret.getValueAsDouble();
            inputs.turretCurrent = currentTurret.getValueAsDouble();
            inputs.turretTemp = tempTurret.getValueAsDouble();
            inputs.turretVelocity = angularVelocityTurret.getValue().in(DegreesPerSecond);
        }

        inputs.wheelConnected = wheelConnectedDebounce.calculate(wheelStatus.isOK());
        inputs.hoodConnected = hoodConnectedDebounce.calculate(hoodStatus.isOK());
        inputs.wheelPosition = positionWheel.getValue().in(Rotations);
        inputs.hoodPosition = positionHood.getValue().in(Degrees);
        inputs.wheelVoltage = voltageWheel.getValueAsDouble();
        inputs.hoodVoltage = voltageHood.getValueAsDouble();
        inputs.wheelCurrent = currentWheel.getValueAsDouble();
        inputs.hoodCurrent = currentHood.getValueAsDouble();
        inputs.wheelTemp = tempWheel.getValueAsDouble();
        inputs.hoodTemp = tempHood.getValueAsDouble();
        inputs.wheelVelocity = angularVelocityWheel.getValue().in(RevolutionsPerSecond) * 60;
        inputs.hoodVelocity = angularVelocityHood.getValue().in(DegreesPerSecond);
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
        if (!hasTurret) return;

        turret.setControl(voltageRequestTurret.withOutput(power * 12));
    }

    @Override
    public void setTurretAngle(double turretAngle, double velocity) {
        if (!hasTurret) return;

        turret.setControl(
                positionRequestTurret
                        .withPosition(Units.degreesToRotations(turretAngle))
                        .withVelocity(Units.degreesToRotations(velocity)));
    }

    @Override
    public void setHoodAngle(double hoodAngle) {
        hood.setControl(positionRequestHood.withPosition(Units.degreesToRotations(hoodAngle)));
    }

    @Override
    public void setSpeed(double speed) {
        wheel.setControl(velocityRequestWheel.withVelocity(speed / 60));
    }
}
