package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
    public static final double armStartWheelPos = 0.07;
    private static final double armOutPos =
            -0.02; // -0.042; // intentionally below zero, actual is -0.013
    private static final double armWontHitTrenchPos = 0.05;
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

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public void extend() {
        io.armMotion(armOutPos);
        overrideToSpinWheels = true;
        Logger.recordOutput("Intake/ArmSetpoint", armOutPos);
    }

    public void retract() {
        io.armMotion(armInPos);
        overrideToSpinWheels = false;
        Logger.recordOutput("Intake/ArmSetpoint", armInPos);
    }

    public Command fastDrop() {
        return new RunCommand(() -> io.armAngle(armOutPos), this)
                .until(() -> inputs.armPosition < armWontHitTrenchPos)
                .finallyDo(this::extend)
                .alongWith(new WaitCommand(1));
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
                    double speed = 3500;

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
        shakeTheIntake.addCommands(new InstantCommand(() -> r.intake.retract()));
        shakeTheIntake.addCommands(r.intake.smartIntake().withTimeout(1.5));
        return shakeTheIntake.repeatedly().finallyDo(() -> r.intake.extend());
    }

    public double getAngle() {
        return Units.rotationsToDegrees(inputs.armPosition);
    }
}
