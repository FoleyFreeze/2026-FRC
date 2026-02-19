package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
    RobotContainer r;
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public Climber(ClimberIO io, RobotContainer r) {
        this.io = io;
        this.r = r;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
        Logger.recordOutput("Climber/Pos", inputs.climb1Position);
        Logger.recordOutput("Climber/Voltage", inputs.climb1Voltage);
    }

    public double getClimbPos1() {
        return inputs.climb1Position;
    }

    public double getClimbPos2() {
        return inputs.climb2Position;
    }

    public Command climbDo1() {
        return new InstantCommand(() -> io.setPower1(1), this);
    }
}
