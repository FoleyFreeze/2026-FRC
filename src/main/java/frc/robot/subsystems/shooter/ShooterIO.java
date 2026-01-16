package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public boolean wheelConnected = false;
        public double wheelVoltage = 0;
        public double wheelCurrent = 0;
        public double wheelPosition = 0;
        public double wheelVelocity = 0;
        public double wheelTemp = 0;

        public boolean hoodConnected = false;
        public double hoodVoltage = 0;
        public double hoodCurrent = 0;
        public double hoodPosition = 0;
        public double hoodVelocity = 0;
        public double hoodTemp = 0;

        public boolean turretConnected = false;
        public double turretVoltage = 0;
        public double turretCurrent = 0;
        public double turretPosition = 0;
        public double turretVelocity = 0;
        public double turretTemp = 0;

        public double turretAbsEnc1 = 0;
        public double turretAbsEnc2 = 0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setAll(double turretAngle, double hoodAngle, double speed) {}

    public default void wheelPower(double power) {}

    public default void wheelTurret(double power) {}

    public default void wheelHood(double power) {}
}
