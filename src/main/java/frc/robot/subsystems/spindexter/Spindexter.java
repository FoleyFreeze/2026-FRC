package frc.robot.subsystems.spindexter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class Spindexter extends SubsystemBase {
    RobotContainer r;

    public static final boolean isDisabled = true;

    double unjam = -0.7;
    double spinPower = 0.7;
    double gatePower = 0.7;

    private final SpindexterIO io;
    private final SpindexterIOInputsAutoLogged inputs = new SpindexterIOInputsAutoLogged();

    public static Spindexter create(RobotContainer r, SpindexterIOSim spinSim) {
        if (isDisabled) {
            return new Spindexter(new SpindexterIO() {}, r);
        }

        switch (Constants.currentMode) {
            case REAL:
                return new Spindexter(new SpindexterIOHardware(), r);
            case SIM:
                return new Spindexter(spinSim, r);
            default:
                return new Spindexter(new SpindexterIO() {}, r);
        }
    }

    public Spindexter(SpindexterIO io, RobotContainer r) {
        this.io = io;
        this.r = r;
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
        return new RunCommand(
                () -> {
                    io.spinPower(0);
                    io.gatePower(0);
                },
                this);
    }

    public Command smartSpinCmd(Shooter shooter, Drive drive) {
        return new RunCommand(() -> smartSpin(), this);
    }

    private void smartSpin() {
        io.spinPower(spinPower);
        if (r.shooter.wontMiss(r.drive.getPose()) && r.drive.wontMiss(r.shooter)) {
            io.gatePower(gatePower);
        } else {
            io.spinPower(0);
        }
    }

    public void unjam() {
        io.spinPower(unjam);
        io.gatePower(unjam);
    }
}
