package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    RobotContainer r;

    public static final boolean isDisabled = false;

    private final IntakeIO io;
    public final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    // positions in rotations
    private static final double armInPos = 0.18;
    private static final double armInShotPos = 0.135;
    public static final double armStartWheelPos = 0.07;
    private static final double armOutPos =
            -0.03; // -0.042; // intentionally below zero, actual is -0.013
    private static final double armVeryOutPos = -0.125;
    private static final double armWontHitTrenchPos = 0.03;
    private static final double armDepotPos = 0.03;
    private static final double armAvoidNetPos = 0.12;
    private static final double armTol = 0.04;

    private static final double wheelSpeed = 1500; // rpm
    private static final double unjamWheelSpeed = -1000;

    private static final double intakeInPower = 0.6;
    private static final double intakeOutPower = -0.5;

    // TODO: set false when we are allowed to use arm again
    private boolean armDisabled = true;
    private boolean overrideToSpinWheels = false;

    public static Intake create(RobotContainer r, IntakeIOSim iis) {
        if (isDisabled) {
            return new Intake(r, new IntakeIO() {});
        }

        switch (Constants.currentMode) {
            case REAL:
                return new Intake(r, new IntakeIOHardware());
            case SIM:
                return new Intake(r, iis);
            default:
                return new Intake(r, new IntakeIO() {});
        }
    }

    public Intake(RobotContainer r, IntakeIO io) {
        this.r = r;
        this.io = io;
    }

    MedianFilter medFilt = new MedianFilter(5);
    double sumFiltCurr = 0;
    Debouncer startupDebounce = new Debouncer(0.1, DebounceType.kRising);

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        double filtCurr = medFilt.calculate(inputs.wheelLCurrent + inputs.wheelRCurrent);

        Logger.recordOutput("Intake/MedFilt", filtCurr);

        boolean startupRaw = Math.abs(velSetpoint - inputs.wheelLVelocity) < 700;
        boolean skipStartup = startupDebounce.calculate(startupRaw);
        Logger.recordOutput("Intake/startupRaw", startupRaw);
        Logger.recordOutput("Intake/skipStart", skipStartup);

        if (skipStartup) {
            sumFiltCurr += Math.max(0, filtCurr - 20) * 0.02;
        } else {
            sumFiltCurr = 0;
        }
        Logger.recordOutput("Intake/sumFiltCurr", sumFiltCurr);
    }

    public void extend() {
        io.armMotion(armOutPos);
        overrideToSpinWheels = true;
        Logger.recordOutput("Intake/ArmSetpoint", armOutPos);
    }

    public void reallyExtend() {
        io.armMotion(armVeryOutPos);
        overrideToSpinWheels = true;
        Logger.recordOutput("Intake/ArmSetpoint", armVeryOutPos);
    }

    public void retract() {
        io.armMotion(armInPos);
        overrideToSpinWheels = false;
        Logger.recordOutput("Intake/ArmSetpoint", armInPos);
    }

    public void retractToDepot() {
        io.armMotion(armDepotPos);
        overrideToSpinWheels = true;
        Logger.recordOutput("Intake/ArmSetpoint", armDepotPos);
    }

    public void extendToAvoidNet() {
        io.armMotion(armAvoidNetPos);
        overrideToSpinWheels = true;
        Logger.recordOutput("Intake/ArmSetpoint", armAvoidNetPos);
    }

    public void retractForShot() {
        io.armMotion(armInShotPos);
        overrideToSpinWheels = false;
        Logger.recordOutput("Intake/ArmSetpoint", armInShotPos);
    }

    public Command fastDrop() {
        return new RunCommand(this::reallyExtend, this)
                .until(() -> inputs.armPosition < armWontHitTrenchPos)
                .andThen(new WaitCommand(0.1))
                .raceWith(new WaitCommand(1));
    }

    public Command runIntake() {
        return new RunCommand(
                () -> {
                    if (!armDisabled || overrideToSpinWheels) {
                        if (inputs.armPosition <= armStartWheelPos) {
                            io.wheelSpeed(wheelSpeed);
                            // io.wheelPower(intakeInPower);
                        } else {
                            io.wheelPower(0);
                        }
                    } else {
                        io.wheelPower(0);
                    }
                },
                this);
    }

    public Command stopIntake() {
        return new InstantCommand(() -> io.wheelPower(0), this);
    }

    public Command dumbIntake() {
        return new RunCommand(() -> io.wheelSpeed(wheelSpeed), this);
        // return new RunCommand(() -> io.wheelPower(intakeInPower), this);
    }

    public Command unjamIntake() {
        return new RunCommand(() -> io.wheelSpeed(unjamWheelSpeed), this);
        // return new RunCommand(() -> io.wheelPower(intakeOutPower), this);
    }

    double velSetpoint = 0;

    public Command velDependentIntake() {
        return new RunCommand(
                () -> {
                    double speed = 4000;

                    // reduce speed by 120rpm per meter per second
                    double reductionRatio = 120;

                    double velReduction =
                            Math.max(0, r.drive.getChassisSpeeds().vxMetersPerSecond)
                                    * reductionRatio;
                    velSetpoint = speed - velReduction;
                    Logger.recordOutput("Intake/VelSepoint", velSetpoint);
                    io.wheelSpeed(velSetpoint);
                    // io.wheelPower(intakeInPower);
                },
                this);
    }

    public Command smartIntake() {
        // using same debounce logic as for shooter unjam
        double waitTimeBeforeUnjam = 1.0;
        double unjamTime = 0.4;
        double rpmOffset = 1000;
        Debouncer intakeDebounce = new Debouncer(waitTimeBeforeUnjam, DebounceType.kRising);

        var intakeSequence = new SequentialCommandGroup();
        intakeSequence.addCommands(new InstantCommand(() -> intakeDebounce.calculate(false)));
        intakeSequence.addCommands(
                velDependentIntake()
                        .until(
                                () ->
                                        intakeDebounce.calculate(
                                                Math.abs(velSetpoint - inputs.wheelLVelocity)
                                                        > rpmOffset)));
        intakeSequence.addCommands(
                unjamIntake().withTimeout(unjamTime).until(() -> inputs.wheelLVelocity < -200));

        return intakeSequence.repeatedly()
        // this uses a higher current limit for open loop control to help it get started
        /*.beforeStarting(
        new RunCommand(() -> io.wheelPower(1))
                .withTimeout(0.5)
                .until(() -> inputs.wheelVelocity > velSetpoint))*/ ;
    }

    public Command shakeTheIntake() {
        SequentialCommandGroup shakeTheIntake = new SequentialCommandGroup();
        shakeTheIntake.addCommands(new InstantCommand(() -> r.intake.extend()));
        shakeTheIntake.addCommands(r.intake.stopIntake());
        shakeTheIntake.addCommands(new WaitCommand(0.35));
        shakeTheIntake.addCommands(new InstantCommand(() -> r.intake.retractForShot()));
        shakeTheIntake.addCommands(r.intake.smartIntake().withTimeout(1.5));
        return shakeTheIntake.repeatedly().finallyDo(() -> r.intake.extend());
    }

    public double getAngle() {
        return Units.rotationsToDegrees(inputs.armPosition);
    }
}
