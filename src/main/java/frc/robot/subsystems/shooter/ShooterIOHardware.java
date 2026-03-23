package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;

public class ShooterIOHardware implements ShooterIO {


    // total angle range of 32.4deg
    public static final double hoodMinAngle = 49.5; // deg
    public static final double hoodMaxAngle = 81.4;
    public static final double hoodMinRot = 0; // rotations
    public static final double hoodMaxRot = 0.0915;

    private final TalonFX wheel;
    private final TalonFX wheel2;
    private final TalonFX hood;
    private TalonFX turret;

    private final CANcoder hoodAbsEnc;
    // private final CANcoder turretAbsEnc1;
    // private final CANcoder turretAbsEnc2;

    private final VoltageOut voltageRequestWheel = new VoltageOut(0);
    private final VoltageOut voltageRequestHood = new VoltageOut(0);
    private VoltageOut voltageRequestTurret = new VoltageOut(0);

    private final MotionMagicVelocityTorqueCurrentFOC velocityRequestWheel =
            new MotionMagicVelocityTorqueCurrentFOC(0);
    private final PositionTorqueCurrentFOC positionRequestHood = new PositionTorqueCurrentFOC(0);
    private PositionTorqueCurrentFOC positionRequestTurret = new PositionTorqueCurrentFOC(0);

    private final StatusSignal<Angle> positionWheel;
    private final StatusSignal<Voltage> voltageWheel;
    private final StatusSignal<Current> currentWheel;
    private final StatusSignal<Temperature> tempWheel;
    private final StatusSignal<AngularVelocity> angularVelocityWheel;
    private final StatusSignal<Current> supplyCurrentWheel;

    private final StatusSignal<Current> currentWheel2;
    private final StatusSignal<Temperature> tempWheel2;
    private final StatusSignal<Current> supplyCurrentWheel2;

    private final StatusSignal<Angle> positionHood;
    private final StatusSignal<Voltage> voltageHood;
    private final StatusSignal<Current> currentHood;
    private final StatusSignal<Temperature> tempHood;
    private final StatusSignal<AngularVelocity> angularVelocityHood;
    private final StatusSignal<Current> supplyCurrentHood;

    private StatusSignal<Angle> positionTurret;
    private StatusSignal<Voltage> voltageTurret;
    private StatusSignal<Current> currentTurret;
    private StatusSignal<Temperature> tempTurret;
    private StatusSignal<AngularVelocity> angularVelocityTurret;
    private StatusSignal<Current> supplyCurrentTurret;

    private final StatusSignal<Angle> positionHoodAbs;

    private final Debouncer wheelConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);
    private final Debouncer hoodConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);
    private final Debouncer turretConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);

    public ShooterIOHardware() {
        wheel = new TalonFX(12, TunerConstants.kCANBus);
        var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.Slot0.kP = 11;
        cfg.Slot0.kS = 5;
        cfg.Slot0.kV = 0.17;
        cfg.TorqueCurrent.PeakForwardTorqueCurrent = 100;
        cfg.TorqueCurrent.PeakReverseTorqueCurrent = -100;
        cfg.MotionMagic.MotionMagicAcceleration = 180;
        cfg.MotionMagic.MotionMagicJerk = 1000;
        wheel.getConfigurator().apply(cfg);

        wheel2 = new TalonFX(13, TunerConstants.kCANBus);
        cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        wheel2.getConfigurator().apply(cfg);
        wheel2.setControl(new Follower(12, MotorAlignmentValue.Opposed));

        hoodAbsEnc = new CANcoder(15, TunerConstants.kCANBus);
        var encCfg = new CANcoderConfiguration();
        encCfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encCfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.8;
        encCfg.MagnetSensor.MagnetOffset = 0.84228515625;
        hoodAbsEnc.getConfigurator().apply(encCfg);

        hood = new TalonFX(15, TunerConstants.kCANBus);
        cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.Slot0.kP = 1800;
        cfg.Slot0.kD = 80;
        cfg.TorqueCurrent.PeakForwardTorqueCurrent = 25;
        cfg.TorqueCurrent.PeakReverseTorqueCurrent = -25;
        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        cfg.Feedback.FeedbackRemoteSensorID = 15;
        cfg.Feedback.RotorToSensorRatio = 9.0 * 48.0 / 20.0;
        cfg.Feedback.SensorToMechanismRatio = 20.0 / 48.0 * 314.0 / 20.0;
        hood.getConfigurator().apply(cfg);

        
            turret = new TalonFX(0);
            // TODO: cfg

            positionTurret = turret.getPosition();
            voltageTurret = turret.getMotorVoltage();
            currentTurret = turret.getTorqueCurrent();
            tempTurret = turret.getDeviceTemp();
            angularVelocityTurret = turret.getVelocity();
            supplyCurrentTurret = turret.getSupplyCurrent();
        

        positionWheel = wheel.getPosition();
        voltageWheel = wheel.getMotorVoltage();
        currentWheel = wheel.getTorqueCurrent();
        tempWheel = wheel.getDeviceTemp();
        angularVelocityWheel = wheel.getVelocity();
        supplyCurrentWheel = wheel.getSupplyCurrent();

        tempWheel2 = wheel2.getDeviceTemp();
        currentWheel2 = wheel2.getTorqueCurrent();
        supplyCurrentWheel2 = wheel2.getSupplyCurrent();

        positionHood = hood.getPosition();
        voltageHood = hood.getMotorVoltage();
        currentHood = hood.getTorqueCurrent();
        tempHood = hood.getDeviceTemp();
        angularVelocityHood = hood.getVelocity();
        supplyCurrentHood = hood.getSupplyCurrent();

        positionHoodAbs = hoodAbsEnc.getAbsolutePosition();


            BaseStatusSignal.setUpdateFrequencyForAll(
                    50,
                    positionTurret,
                    voltageTurret,
                    currentTurret,
                    tempTurret,
                    angularVelocityTurret,
                    supplyCurrentTurret);
            ParentDevice.optimizeBusUtilizationForAll(turret);
        

        BaseStatusSignal.setUpdateFrequencyForAll(200, currentWheel);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                positionWheel,
                positionHood,
                voltageWheel,
                voltageHood,
                currentHood,
                tempWheel,
                tempHood,
                angularVelocityWheel,
                angularVelocityHood,
                positionHoodAbs,
                currentWheel2,
                tempWheel2,
                supplyCurrentHood,
                supplyCurrentWheel,
                supplyCurrentWheel2);
        ParentDevice.optimizeBusUtilizationForAll(wheel, wheel2, hood, hoodAbsEnc);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        StatusCode wheelStatus =
                BaseStatusSignal.refreshAll(
                        positionWheel,
                        voltageWheel,
                        currentWheel,
                        tempWheel,
                        angularVelocityWheel,
                        tempWheel2,
                        currentWheel2,
                        supplyCurrentWheel,
                        supplyCurrentWheel2);
        StatusCode hoodStatus =
                BaseStatusSignal.refreshAll(
                        positionHood,
                        voltageHood,
                        currentHood,
                        tempHood,
                        angularVelocityHood,
                        positionHoodAbs,
                        supplyCurrentHood);

            StatusCode turretStatus =
                    BaseStatusSignal.refreshAll(
                            positionTurret,
                            voltageTurret,
                            currentTurret,
                            tempTurret,
                            angularVelocityTurret,
                            supplyCurrentTurret);
            inputs.turretConnected = turretConnectedDebounce.calculate(turretStatus.isOK());
            inputs.turretPositionDeg = positionTurret.getValue().in(Degrees);
            inputs.turretVoltage = voltageTurret.getValueAsDouble();
            inputs.turretCurrent = currentTurret.getValueAsDouble();
            inputs.turretTemp = tempTurret.getValueAsDouble();
            inputs.turretVelocity = angularVelocityTurret.getValue().in(DegreesPerSecond);
            inputs.turretSupplyCurrent = supplyCurrentTurret.getValueAsDouble();

        inputs.wheelConnected = wheelConnectedDebounce.calculate(wheelStatus.isOK());
        inputs.hoodConnected = hoodConnectedDebounce.calculate(hoodStatus.isOK());
        inputs.wheelPosition = positionWheel.getValue().in(Rotations);
        inputs.wheelVoltage = voltageWheel.getValueAsDouble();
        inputs.hoodVoltage = voltageHood.getValueAsDouble();
        inputs.wheelCurrent = currentWheel.getValueAsDouble();
        inputs.hoodCurrent = currentHood.getValueAsDouble();
        inputs.wheelTemp = tempWheel.getValueAsDouble();
        inputs.hoodTemp = tempHood.getValueAsDouble();
        inputs.wheelVelocityRPM = angularVelocityWheel.getValue().in(RevolutionsPerSecond) * 60;
        inputs.hoodVelocity = angularVelocityHood.getValue().in(DegreesPerSecond);
        inputs.hoodAbsEncRotations = positionHoodAbs.getValueAsDouble();
        inputs.wheel2Temp = tempWheel2.getValueAsDouble();
        inputs.wheel2Current = currentWheel2.getValueAsDouble();

        inputs.wheelSupplyCurrent = supplyCurrentWheel.getValueAsDouble();
        inputs.wheel2SupplyCurrent = supplyCurrentWheel2.getValueAsDouble();
        inputs.hoodSupplyCurrent = supplyCurrentHood.getValueAsDouble();

        double rawHoodPosition = positionHood.getValue().in(Rotations);
        inputs.hoodPositionRaw = rawHoodPosition;
        double t = MathUtil.inverseInterpolate(hoodMinRot, hoodMaxRot, rawHoodPosition);
        inputs.hoodPositionDeg = MathUtil.interpolate(hoodMinAngle, hoodMaxAngle, t);
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
    public void setTurretAngle(double turretAngle, double velocity) {
        

        turret.setControl(
                positionRequestTurret
                        .withPosition(Units.degreesToRotations(turretAngle))
                        .withVelocity(Units.degreesToRotations(velocity)));
    }

    @Override
    public void setHoodAngle(double hoodAngle) {
        double t = MathUtil.inverseInterpolate(hoodMinAngle, hoodMaxAngle, hoodAngle);
        t = MathUtil.clamp(t, 0, 1);
        double rotationSetPoint = MathUtil.interpolate(hoodMinRot, hoodMaxRot, t);

        // hood.setControl(positionRequestHood.withPosition(Units.degreesToRotations(hoodAngle)));
        hood.setControl(positionRequestHood.withPosition(rotationSetPoint));
    }

    @Override
    public void setSpeed(double speed) {
        wheel.setControl(velocityRequestWheel.withVelocity(speed / 60));
    }
}
