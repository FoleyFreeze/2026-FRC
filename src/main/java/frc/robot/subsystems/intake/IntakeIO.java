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

        public double armPositionAbs = 0;

        public boolean wheelConnected = false;
        public double wheelLVoltage = 0;
        public double wheelLCurrent = 0;
        public double wheelLPosition = 0;
        public double wheelLVelocity = 0;
        public double wheelLTemp = 0;

        public double wheelRCurrent = 0;
        public double wheelRTemp = 0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void armPower(double power) {}

    public default void armAngle(double rotations) {}

    public default void armMotion(double rotations) {}

    public default void wheelSpeed(double speed) {}

    public default void wheelPower(double Power) {}
}
