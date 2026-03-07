package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    RobotContainer r;

    public static final boolean isDisabled = false;

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    // positions in rotations
    private static final double armInPos = 0.25;
    private static final double armStartWheelPos = 0.11;
    private static final double armOutPos = 0.005;
    private static final double armTol = 0.04;

    private static final double wheelSpeed = 2000; // rpm

    // TODO: set false when we are allowed to use arm again
    private boolean armDisabled = true;
    private boolean overrideToSpinWheels = false;

    public static Intake create(RobotContainer r, IntakeIOSim iis) {
        if (isDisabled) {
            return new Intake(new IntakeIO() {});
        }

        switch (Constants.currentMode) {
            case REAL:
                return new Intake(new IntakeIOHardware());
            case SIM:
                return new Intake(iis);
            default:
                return new Intake(new IntakeIO() {});
        }
    }

    public Intake(IntakeIO io) {
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
    }

    public void retract() {
        io.armMotion(armInPos);
        overrideToSpinWheels = false;
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

    public double getAngle() {
        return Units.rotationsToDegrees(inputs.armPosition);
    }
}
