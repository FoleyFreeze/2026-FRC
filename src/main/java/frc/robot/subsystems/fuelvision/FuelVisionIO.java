package frc.robot.subsystems.fuelvision;

import org.littletonrobotics.junction.AutoLog;

public interface FuelVisionIO {

    @AutoLog
    public static class FuelVisionIOInputs {
        int seqNum;
        float rioTime;
        float imageTime;
        FuelVisionData[] fuelData;
        double now;
    }

    public default void updateInputs(FuelVisionIOInputs inputs) {}

    public default void fuelCamEnable(boolean enable) {}
}
