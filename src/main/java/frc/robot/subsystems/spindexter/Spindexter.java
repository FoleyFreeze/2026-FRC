package frc.robot.subsystems.spindexter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class Spindexter extends SubsystemBase {
    private final SpindexterIO io;
    private final SpindexterIOInputsAutoLogged inputs = new SpindexterIOInputsAutoLogged();

    public Spindexter(SpindexterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Spindexter", inputs);
    }

    public Command spin() {
        return new RunCommand(() -> io.spinPower(1), this);
    }

    public Command stop() {
        return new RunCommand(() -> io.spinPower(0), this);
    }

    public Command smartSpinCmd(Shooter shooter, Drive drive) {
        return new RunCommand(() -> smartSpin(shooter, drive), this);
    }

    private void smartSpin(Shooter shooter, Drive drive) {
        if (shooter.wontMiss() && drive.wontMiss()) {
            io.spinPower(1);
        } else {
            io.spinPower(0);
        }
    }
}
