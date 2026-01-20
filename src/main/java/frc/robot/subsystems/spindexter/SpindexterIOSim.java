package frc.robot.subsystems.spindexter;

public class SpindexterIOSim implements SpindexterIO {
    public boolean areWeRunning = false;

    public void updateInputs(SpindexterIOInputs inputs) {
        inputs.spinConnected = true;
        inputs.spinCurrent = 0;
        inputs.spinPosition = 0;
        inputs.spinTemp = 0;
        inputs.spinVelocity = 0;
        inputs.spinVoltage = 0;

        if (areWeRunning) {
            inputs.spinVoltage = 12;
        } else {
            inputs.spinVoltage = 0;
        }
    }

    public void spinSpeed(double speed) {
        areWeRunning = speed > 0;
    }

    public void spinPower(double power) {
        areWeRunning = power > 0;
    }
}
