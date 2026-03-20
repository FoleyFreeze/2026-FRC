package frc.robot.subsystems.fuelvision;

import org.littletonrobotics.junction.AutoLog;

public interface FuelVisionIO {

    @AutoLog
    public static class FuelVisionIOInputs {
        int seqNum = 0;
        float rioTime = 0;
        float imageTime = 0;
        FuelVisionData[] fuelData = new FuelVisionData[0];
        double now = 0;
    }

    public default void updateInputs(FuelVisionIOInputs inputs) {}

    public default void fuelCamEnable(boolean enable) {}
}
