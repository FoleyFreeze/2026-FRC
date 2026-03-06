package frc.robot.subsystems.spindexter;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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

    public static final boolean isDisabled = false;

    double unjam = -0.7;
    double spinPower = 0.7;
    double gatePower = 0.7;
    double gateSpeed = 2000;
    double spinSpeed = 1500;

    private final SpindexterIO io;
    private final SpindexterIOInputsAutoLogged inputs = new SpindexterIOInputsAutoLogged();

    NetworkTableEntry gateSet =
            NetworkTableInstance.getDefault().getTable("Tuning").getEntry("GateSet");
    NetworkTableEntry spinSet =
            NetworkTableInstance.getDefault().getTable("Tuning").getEntry("SpinSet");

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

        gateSet.setDouble(gateSpeed);
        gateSet.getDouble(0);
        spinSet.setDouble(spinSpeed);
        spinSet.getDouble(0);
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
        // io.gateSpeed(gateSpeed);
        double gateSetpoint = gateSet.getDouble(gateSpeed);
        io.gateSpeed(gateSetpoint);
        if (r.shooter.wontMiss(r.drive.getPose()) && r.drive.wontMiss(r.shooter)) {
            if (Math.abs(inputs.gateVelocity - gateSetpoint) < 100) {
                io.spinSpeed(spinSet.getDouble(spinSpeed));
            } else {
                io.spinPower(0);
            }
        } else {
            io.spinPower(0);
        }
    }

    public void unjam() {
        io.spinPower(unjam);
        io.gatePower(unjam);
    }
}
