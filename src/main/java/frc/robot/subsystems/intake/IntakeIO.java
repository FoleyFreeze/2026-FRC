package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public boolean armConnected = false;
        public double armVoltage = 0;
        public double armCurrent = 0;
        public double armPosition = 0;
        public double armVelocity = 0;
        public double armTemp = 0;

        public boolean wheelConnected = false;
        public double wheelVoltage = 0;
        public double wheelCurrent = 0;
        public double wheelPosition = 0;
        public double wheelVelocity = 0;
        public double wheelTemp = 0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void armPower(double power) {}

    public default void armAngle(double angle) {}
    
    public default void armMotion(double angle) {}

    public default void wheelSpeed(double speed) {}

    public default void wheelPower(double Power) {}
}
