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
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.PhoenixUtil;
import org.littletonrobotics.junction.Logger;

public class ShooterIOHardware implements ShooterIO {

    // total angle range of 32.4deg
    public static final double hoodMinAngle = 49.5; // deg
    public static final double hoodMaxAngle = 81.4;
    public static final double hoodMinRot = 0.0261; // 0.0; // rotations
    public static final double hoodMaxRot = 0.1174; // 0.0915;

    public static final double turretMaxTemp = 65; // deg C
    public static final double turretMaxTempHyst = 60;
    private boolean pausedDueToTurretTemp = false;
    private Alert turretTempAlert = new Alert("Turret Temp Too High!", AlertType.kError);

    private final TalonFX wheel;
    private final TalonFX wheel2;
    private final TalonFX hood;
    private final TalonFX turret;

    private final CANcoder hoodAbsEnc;
    private final CANcoder turretAbsEnc27;
    private final CANcoder turretAbsEnc29;

    private final VoltageOut voltageRequestWheel = new VoltageOut(0);
    private final VoltageOut voltageRequestHood = new VoltageOut(0);
    private final VoltageOut voltageRequestTurret = new VoltageOut(0);

    private final MotionMagicVelocityTorqueCurrentFOC velocityRequestWheel =
            new MotionMagicVelocityTorqueCurrentFOC(0);
    private final PositionTorqueCurrentFOC positionRequestHood = new PositionTorqueCurrentFOC(0);
    private final MotionMagicTorqueCurrentFOC positionMagicRequestTurret =
            new MotionMagicTorqueCurrentFOC(0);
    private final PositionTorqueCurrentFOC positionRequestTurret =
            new PositionTorqueCurrentFOC(0).withSlot(1);

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

    private final StatusSignal<Angle> positionTurret;
    private final StatusSignal<Voltage> voltageTurret;
    private final StatusSignal<Current> currentTurret;
    private final StatusSignal<Temperature> tempTurret;
    private final StatusSignal<AngularVelocity> angularVelocityTurret;
    private final StatusSignal<Current> supplyCurrentTurret;

    private final StatusSignal<Angle> positionHoodAbs;
    private final StatusSignal<Angle> enc27Abs;
    private final StatusSignal<Angle> enc29Abs;

    private final Debouncer wheelConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);
    private final Debouncer hoodConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);
    private final Debouncer turretConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);

    public static final double turretGearRatio = 5.0 * 172.0 / 45.0; // rotor to mechanism rotations
    private double lastTurretAngle = 0;

    public ShooterIOHardware() {
        wheel = new TalonFX(12, TunerConstants.kCANBus);
        var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.Slot0.kP = 10;
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
        var encCfgH = new CANcoderConfiguration();
        encCfgH.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encCfgH.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.8;
        encCfgH.MagnetSensor.MagnetOffset = 0.84228515625;
        PhoenixUtil.tryUntilOk(5, () -> hoodAbsEnc.getConfigurator().apply(encCfgH));

        hood = new TalonFX(15, TunerConstants.kCANBus);
        var cfgH = new TalonFXConfiguration();
        cfgH.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfgH.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfgH.Slot0.kP = 1800;
        cfgH.Slot0.kD = 80;
        cfgH.TorqueCurrent.PeakForwardTorqueCurrent = 25;
        cfgH.TorqueCurrent.PeakReverseTorqueCurrent = -25;
        cfgH.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        cfgH.Feedback.FeedbackRemoteSensorID = 15;
        cfgH.Feedback.RotorToSensorRatio = 9.0 * 48.0 / 20.0;
        cfgH.Feedback.SensorToMechanismRatio = 20.0 / 48.0 * 314.0 / 20.0;
        PhoenixUtil.tryUntilOk(5, () -> hood.getConfigurator().apply(cfgH));

        turretAbsEnc27 = new CANcoder(27, TunerConstants.kCANBus);
        var encCfg27 = new CANcoderConfiguration();
        encCfg27.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encCfg27.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        encCfg27.MagnetSensor.MagnetOffset = -0.8164;
        PhoenixUtil.tryUntilOk(5, () -> turretAbsEnc27.getConfigurator().apply(encCfg27));

        turretAbsEnc29 = new CANcoder(29, TunerConstants.kCANBus);
        var encCfg29 = new CANcoderConfiguration();
        encCfg29.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encCfg29.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        encCfg29.MagnetSensor.MagnetOffset = -0.0461;
        PhoenixUtil.tryUntilOk(5, () -> turretAbsEnc29.getConfigurator().apply(encCfg29));

        turret = new TalonFX(7, TunerConstants.kCANBus);
        var cfgT = new TalonFXConfiguration();
        cfgT.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfgT.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfgT.Slot0.kP = 5000;
        cfgT.Slot0.kD = 35;
        cfgT.Slot0.kS = 0;
        cfgT.Slot0.kV = 0;
        cfgT.Slot0.kA = 0;
        cfgT.Slot1.kP = 5000;
        cfgT.Slot1.kD = 35;
        cfgT.Slot1.kS = 0;
        cfgT.Slot1.kV = 0;
        cfgT.Slot1.kA = 0;
        cfgT.TorqueCurrent.PeakForwardTorqueCurrent = 100;
        cfgT.TorqueCurrent.PeakReverseTorqueCurrent = -100;
        cfgT.MotionMagic.MotionMagicAcceleration = 2.25;
        cfgT.MotionMagic.MotionMagicCruiseVelocity = 1.75;
        cfgT.Feedback.SensorToMechanismRatio = turretGearRatio;
        PhoenixUtil.tryUntilOk(5, () -> turret.getConfigurator().apply(cfgT));

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
        enc27Abs = turretAbsEnc27.getAbsolutePosition();
        enc29Abs = turretAbsEnc29.getAbsolutePosition();

        // things I want fast
        // TODO: temporary?
        BaseStatusSignal.setUpdateFrequencyForAll(
                Drive.ODOMETRY_FREQUENCY,
                currentWheel,
                angularVelocityWheel,
                currentTurret,
                positionTurret);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                positionWheel,
                positionHood,
                voltageWheel,
                voltageHood,
                currentHood,
                tempWheel,
                tempHood,
                // angularVelocityWheel,
                angularVelocityHood,
                positionHoodAbs,
                // currentWheel,
                currentWheel2,
                tempWheel2,
                supplyCurrentHood,
                supplyCurrentWheel,
                supplyCurrentWheel2,
                // positionTurret,
                voltageTurret,
                // currentTurret,
                tempTurret,
                angularVelocityTurret,
                supplyCurrentTurret,
                enc27Abs,
                enc29Abs);
        ParentDevice.optimizeBusUtilizationForAll(
                wheel, wheel2, hood, hoodAbsEnc, turret, turretAbsEnc27, turretAbsEnc29);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        StatusCode status =
                BaseStatusSignal.refreshAll(
                        positionWheel,
                        voltageWheel,
                        currentWheel,
                        tempWheel,
                        angularVelocityWheel,
                        tempWheel2,
                        currentWheel2,
                        supplyCurrentWheel,
                        supplyCurrentWheel2,
                        positionHood,
                        voltageHood,
                        currentHood,
                        tempHood,
                        angularVelocityHood,
                        positionHoodAbs,
                        supplyCurrentHood,
                        positionTurret,
                        voltageTurret,
                        currentTurret,
                        tempTurret,
                        angularVelocityTurret,
                        supplyCurrentTurret,
                        enc27Abs,
                        enc29Abs);

        inputs.turretConnected =
                turretConnectedDebounce.calculate(positionTurret.getStatus().isOK());
        inputs.turretPositionDeg = positionTurret.getValue().in(Degrees);
        inputs.turretVoltage = voltageTurret.getValueAsDouble();
        inputs.turretCurrent = currentTurret.getValueAsDouble();
        inputs.turretTemp = tempTurret.getValueAsDouble();
        inputs.turretVelocity = angularVelocityTurret.getValue().in(DegreesPerSecond);
        inputs.turretSupplyCurrent = supplyCurrentTurret.getValueAsDouble();

        inputs.wheelConnected =
                wheelConnectedDebounce.calculate(
                        positionWheel.getStatus().isOK() && supplyCurrentWheel2.getStatus().isOK());
        inputs.hoodConnected = hoodConnectedDebounce.calculate(positionHood.getStatus().isOK());
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

        inputs.turretAbsEnc27Deg = enc27Abs.getValue().in(Degrees);
        inputs.turretAbsEnc29Deg = enc29Abs.getValue().in(Degrees);

        // latch off when temp is high until it falls below the lower limit
        if (pausedDueToTurretTemp) {
            pausedDueToTurretTemp = inputs.turretTemp < turretMaxTempHyst;
        } else {
            pausedDueToTurretTemp = inputs.turretTemp > turretMaxTemp;
        }
        turretTempAlert.set(pausedDueToTurretTemp);

        lastTurretAngle = inputs.turretPositionDeg;
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
        if (pausedDueToTurretTemp) {
            // stop turret when its hot
            turret.setControl(voltageRequestTurret.withOutput(0));
        } else {
            if (Math.abs(turretAngle - lastTurretAngle) > 15) {
                // ignoring velocity because motion magic
                turret.setControl(
                        positionMagicRequestTurret.withPosition(
                                Units.degreesToRotations(turretAngle)));
            } else {
                turret.setControl(
                        positionRequestTurret
                                .withPosition(Units.degreesToRotations(turretAngle))
                                .withVelocity(Units.degreesToRotations(velocity)));
            }
        }
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

    @Override
    public void zeroTurretToEnc() {
        var status = BaseStatusSignal.refreshAll(enc27Abs, enc29Abs);
        if (status.isOK()) {
            double turretAngleOffset =
                    Shooter.getAngleCRT(
                            enc27Abs.getValue().in(Degrees), enc29Abs.getValue().in(Degrees));
            turret.setPosition((turretAngleOffset - Constants.turretAngleOffset) / 360.0);
            Logger.recordOutput("Shooter/TurretZero", turretAngleOffset);
        } else {
            Logger.recordOutput("Shooter/ZeroFailureReason", status.toString());
        }
    }
}
