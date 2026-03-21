package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;
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
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;

public class IntakeIOHardware implements IntakeIO {

    private final TalonFX wheelL;
    private final TalonFX wheelR;
    private final TalonFX intakeBar;
    private final CANcoder intakeAbsEnc;

    // NOTE: disabled FOC on intake wheel based on this thread:
    // https://www.chiefdelphi.com/t/kraken-x60-limp-mode-behavior/515080/95
    private final VoltageOut voltageRequestWheel = new VoltageOut(0).withEnableFOC(false);
    private final VoltageOut voltageRequestArm = new VoltageOut(0);

    private final MotionMagicVelocityTorqueCurrentFOC velocityRequestWheel =
            new MotionMagicVelocityTorqueCurrentFOC(0);
    private final PositionTorqueCurrentFOC positionRequestArm =
            new PositionTorqueCurrentFOC(0).withSlot(0);
    private final MotionMagicTorqueCurrentFOC motionRequestArm =
            new MotionMagicTorqueCurrentFOC(0).withSlot(1);

    private final StatusSignal<Angle> positionWheelL;
    private final StatusSignal<Angle> positionArm;
    private final StatusSignal<Angle> positionArmAbs;
    private final StatusSignal<Voltage> voltageWheelL;
    private final StatusSignal<Voltage> voltageArm;
    private final StatusSignal<Current> currentWheelL;
    private final StatusSignal<Current> currentArm;
    private final StatusSignal<Temperature> tempWheelL;
    private final StatusSignal<Temperature> tempArm;
    private final StatusSignal<AngularVelocity> angularVelocityWheelL;
    private final StatusSignal<AngularVelocity> angularVelocityArm;

    private final StatusSignal<Current> currentWheelR;
    private final StatusSignal<Temperature> tempWheelR;

    private final Debouncer wheelConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);
    private final Debouncer armConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);

    public IntakeIOHardware() {
        wheelL = new TalonFX(3, TunerConstants.kCANBus);
        var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.Slot0.kP = 12;
        cfg.Slot0.kS = 4;
        cfg.Slot0.kV = 0.01;
        cfg.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        cfg.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        cfg.MotionMagic.MotionMagicAcceleration = 600;
        cfg.MotionMagic.MotionMagicJerk = 0;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = 60;
        wheelL.getConfigurator().apply(cfg);

        wheelR = new TalonFX(16, TunerConstants.kCANBus);
        cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        cfg.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = 60;
        wheelR.getConfigurator().apply(cfg);
        wheelR.setControl(new Follower(3, MotorAlignmentValue.Opposed));

        intakeAbsEnc = new CANcoder(2, TunerConstants.kCANBus);
        var encCfg = new CANcoderConfiguration();
        encCfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encCfg.MagnetSensor.MagnetOffset = -0.67;
        encCfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.7;
        intakeAbsEnc.getConfigurator().apply(encCfg);

        intakeBar = new TalonFX(2, TunerConstants.kCANBus);
        cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.TorqueCurrent.PeakForwardTorqueCurrent = 70;
        cfg.TorqueCurrent.PeakReverseTorqueCurrent = -70;
        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        cfg.Feedback.FeedbackRemoteSensorID = 2;
        cfg.Feedback.RotorToSensorRatio = 52.0 / 16.0 * 52.0 / 24.0 * 54.0 / 18.0;
        cfg.Feedback.SensorToMechanismRatio = 38.0 / 22.0;
        cfg.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
        cfg.Slot1.GravityArmPositionOffset = 0.0;
        cfg.Slot1.kG = 0;
        cfg.Slot1.kP = 800;
        cfg.Slot1.kD = 50;
        cfg.Slot1.kV = 0;
        cfg.Slot1.kA = 30;
        cfg.MotionMagic.MotionMagicCruiseVelocity = 0.5;
        cfg.MotionMagic.MotionMagicAcceleration = 1;
        cfg.Slot0.kP = 500;
        cfg.Slot0.kD = 120;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = 70;
        intakeBar.getConfigurator().apply(cfg);

        positionWheelL = wheelL.getPosition();
        positionArm = intakeBar.getPosition();
        voltageWheelL = wheelL.getMotorVoltage();
        voltageArm = intakeBar.getMotorVoltage();
        currentWheelL = wheelL.getStatorCurrent();
        currentArm = intakeBar.getStatorCurrent();
        tempWheelL = wheelL.getDeviceTemp();
        tempArm = intakeBar.getDeviceTemp();
        angularVelocityWheelL = wheelL.getVelocity();
        angularVelocityArm = intakeBar.getVelocity();
        positionArmAbs = intakeAbsEnc.getPosition();

        currentWheelR = wheelR.getTorqueCurrent();
        tempWheelR = wheelR.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                positionWheelL,
                positionArm,
                voltageWheelL,
                voltageArm,
                currentWheelL,
                currentArm,
                tempWheelL,
                tempArm,
                angularVelocityWheelL,
                angularVelocityArm,
                positionArmAbs,
                currentWheelR,
                tempWheelR);
        ParentDevice.optimizeBusUtilizationForAll(wheelL, wheelR, intakeBar, intakeAbsEnc);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        StatusCode wheelStatus =
                BaseStatusSignal.refreshAll(
                        positionWheelL,
                        voltageWheelL,
                        currentWheelL,
                        tempWheelL,
                        angularVelocityWheelL,
                        currentWheelR,
                        tempWheelR);
        StatusCode armStatus =
                BaseStatusSignal.refreshAll(
                        positionArm,
                        voltageArm,
                        currentArm,
                        tempArm,
                        angularVelocityArm,
                        positionArmAbs);
        inputs.wheelConnected = wheelConnectedDebounce.calculate(wheelStatus.isOK());
        inputs.armConnected = armConnectedDebounce.calculate(armStatus.isOK());
        inputs.wheelLPosition = positionWheelL.getValue().in(Rotations);
        inputs.armPosition = positionArm.getValue().in(Rotations);
        inputs.wheelLVoltage = voltageWheelL.getValueAsDouble();
        inputs.armVoltage = voltageArm.getValueAsDouble();
        inputs.wheelLCurrent = currentWheelL.getValueAsDouble();
        inputs.armCurrent = currentArm.getValueAsDouble();
        inputs.wheelLTemp = tempWheelL.getValueAsDouble();
        inputs.armTemp = tempArm.getValueAsDouble();
        inputs.wheelLVelocity = angularVelocityWheelL.getValue().in(RPM);
        inputs.armVelocity = angularVelocityArm.getValue().in(RPM);
        inputs.armPositionAbs = positionArmAbs.getValue().in(Rotations);

        inputs.wheelRCurrent = currentWheelR.getValueAsDouble();
        inputs.wheelRTemp = tempWheelR.getValueAsDouble();
    }

    @Override
    public void wheelPower(double power) {
        wheelL.setControl(voltageRequestWheel.withOutput(power * 12));
    }

    @Override
    public void wheelSpeed(double speed) {
        wheelL.setControl(velocityRequestWheel.withVelocity(speed / 60));
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
