package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;

        // TODO: remove once we have position control
        setDefaultCommand(new RunCommand(() -> io.armPower(0), this));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public double getAngle() {
        return inputs.armPosition;
    }

    public Command intakeUp() {
        return new RunCommand(() -> io.armPower(-1), this);
    }

    public Command intakeDown() {
        return new RunCommand(() -> io.armPower(1), this);
    }
}
