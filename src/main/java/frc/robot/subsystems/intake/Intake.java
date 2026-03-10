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
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    // positions in rotations
    private static final double armInPos = 0.09; // 0.18;
    private static final double armStartWheelPos = 0.61;
    private static final double armOutPos = -0.019; // intentionally below zero
    private static final double armTol = 0.04;

    private static final double wheelSpeed = 1500; // rpm
    private static final double unjamWheelSpeed = -1000;

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

    public Command runIntake() {
        return new RunCommand(
                () -> {
                    if (!armDisabled || overrideToSpinWheels) {
                        if (inputs.armPosition <= armStartWheelPos) {
                            io.wheelSpeed(wheelSpeed);
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
    }

    public Command unjamIntake() {
        return new RunCommand(() -> io.wheelSpeed(unjamWheelSpeed), this);
    }

    double velSetpoint = 0;

    public Command velDependentIntake() {
        return new RunCommand(
                () -> {
                    double speed = 4400;

                    // reduce speed by 120rpm per meter per second
                    double reductionRatio = 120;

                    double velReduction =
                            Math.max(0, r.drive.getChassisSpeeds().vxMetersPerSecond)
                                    * reductionRatio;
                    velSetpoint = speed - velReduction;
                    Logger.recordOutput("Intake/VelSepoint", velSetpoint);
                    io.wheelSpeed(velSetpoint);
                },
                this);
    }

    public Command smartIntake() {
        // using same debounce logic as for shooter unjam
        double waitTimeBeforeUnjam = 0.75;
        double unjamTime = 0.75;
        double rpmOffset = 400;
        Debouncer intakeDebounce = new Debouncer(waitTimeBeforeUnjam, DebounceType.kRising);

        var intakeSequence = new SequentialCommandGroup();
        intakeSequence.addCommands(new InstantCommand(() -> intakeDebounce.calculate(false)));
        intakeSequence.addCommands(
                velDependentIntake()
                        .until(
                                () ->
                                        intakeDebounce.calculate(
                                                Math.abs(velSetpoint - inputs.wheelVelocity)
                                                        > rpmOffset)));
        intakeSequence.addCommands(unjamIntake().withDeadline(new WaitCommand(unjamTime)));

        return intakeSequence.repeatedly();
    }

    public double getAngle() {
        return Units.rotationsToDegrees(inputs.armPosition);
    }
}
