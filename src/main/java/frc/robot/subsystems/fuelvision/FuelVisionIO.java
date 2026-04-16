package frc.robot.subsystems.fuelvision;

import org.littletonrobotics.junction.AutoLog;

public interface FuelVisionIO {

    @AutoLog
    public static class FuelVisionIOInputs {
        public int seqNum = 0;
        public float rioTime = 0;
        public float imageTime = 0;
        public float realTime = 0;
        public FuelVisionData[] fuelData = new FuelVisionData[0];
        public double now = 0;
    }

    public default void updateInputs(FuelVisionIOInputs inputs) {}

    public default void fuelCamEnable(boolean enable) {}
}
