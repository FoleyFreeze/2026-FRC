package frc.robot.subsystems.spindexter;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexterIO {
    @AutoLog
    public static class SpindexterIOInputs {
        public boolean gateConnected = false;
        public double gateVoltage = 0;
        public double gateCurrent = 0;
        public double gatePosition = 0;
        public double gateVelocity = 0;
        public double gateTemp = 0;

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

    public default void gatePower(double power) {}

    public default void gateSpeed(double speed) {}
}
