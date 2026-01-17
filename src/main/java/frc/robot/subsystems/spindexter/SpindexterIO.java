package frc.robot.subsystems.spindexter;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexterIO {
    @AutoLog
    public static class SpindexterIOInputs {
        public boolean spinConnected = false;
        public double spinVoltage = 0;
        public double spinCurrent = 0;
        public double spinPosition = 0;
        public double spinVelocity = 0;
        public double spinTemp = 0;
    }

    public default void updateInputs(SpindexterIOInputs inputs) {}

    public default void spinSpeed(double speed) {}

    public default void spinPower(double power) {}
}
