package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    RobotContainer r;

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public enum IntakeMode {
        EXTENDING,
        RETRACTING,
        HOLD_OUT,
        HOLD_IN
    }

    private IntakeMode mode = IntakeMode.HOLD_IN;

    // positions in rotations
    private static final double armInPos = 0;
    private static final double armOutPos = 0.25; // TODO: measure real value
    private static final double wheelSpeed = 0.7;
    private static final double armTol = 0.02;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        // control intake arm via main state machine
        switch (mode) {
            case EXTENDING:
                io.armMotion(armOutPos);
                io.wheelPower(0);
                if (Math.abs(inputs.armPosition - armOutPos) < armTol) {
                    mode = IntakeMode.HOLD_OUT;
                }
                break;
            case RETRACTING:
                io.armMotion(armInPos);
                io.wheelPower(0);
                if (Math.abs(inputs.armPosition - armInPos) < armTol) {
                    mode = IntakeMode.HOLD_IN;
                }
                break;

            case HOLD_IN:
                io.armAngle(armInPos);
                io.wheelPower(0);
                break;
            case HOLD_OUT:
                io.armAngle(armOutPos);
                io.wheelPower(wheelSpeed);
                break;
        }

        Logger.recordOutput("Intake/Mode", mode);
    }

    public void extend() {
        mode = IntakeMode.EXTENDING;
    }

    public void retract() {
        mode = IntakeMode.RETRACTING;
    }

    public double getAngle() {
        return Units.rotationsToDegrees(inputs.armPosition);
    }
}
