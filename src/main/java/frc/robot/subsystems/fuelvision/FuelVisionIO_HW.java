package frc.robot.subsystems.fuelvision;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import java.nio.ByteBuffer;
import java.util.EnumSet;

public class FuelVisionIO_HW implements FuelVisionIO {
    private RawSubscriber poseMsgFuel;
    private ByteBuffer poseDataFuel;

    private BooleanEntry fuelActive;

    public static class FuelVisionHeader {
        int seqNum;
        float rioTime;
        float imageTime;
        float realTime;
    }

    FuelVisionHeader fuelHeader = new FuelVisionHeader();
    FuelVisionData[] fuelData = new FuelVisionData[0];

    // based on the 2023-FRC project
    public FuelVisionIO_HW() {
        fuelActive =
                NetworkTableInstance.getDefault()
                        .getBooleanTopic("/Vision/Fuel Enable")
                        .getEntry(true);
        fuelActive.set(true);

        poseMsgFuel =
                NetworkTableInstance.getDefault()
                        .getTable("Vision")
                        .getRawTopic("Fuel Pose Data Bytes")
                        .subscribe("raw", null);

        NetworkTableInstance.getDefault()
                .addListener(
                        poseMsgFuel,
                        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                        (event) -> {
                            poseDataFuel = ByteBuffer.wrap(event.valueData.value.getRaw());

                            FuelVisionHeader fvh = new FuelVisionHeader();

                            // parse header
                            float currentTime = (float) Timer.getFPGATimestamp();
                            fvh.seqNum = poseDataFuel.getInt();
                            fvh.rioTime = poseDataFuel.getFloat();
                            fvh.imageTime = poseDataFuel.getFloat();
                            int length = Byte.toUnsignedInt(poseDataFuel.get());

                            // convert into robot timestamp (time since rio boot)
                            // time image was taken is currentTime - latency/2 - imageProcTime
                            // where latency is currentTime - rioTime
                            // (the last recieved rio timestamp by the pi at the time the pi sent
                            // this message)
                            // where imageProcTime is imageTime and is the time between when the
                            // frame was captured to when data was sent
                            fvh.realTime =
                                    currentTime
                                            - (currentTime - fvh.rioTime) / 2.0f
                                            - fvh.imageTime;

                            // parse fuel groups
                            FuelVisionData[] fuelArray = new FuelVisionData[length];
                            for (int i = 0; i < length; i++) {
                                FuelVisionData fvd = new FuelVisionData();
                                fvd.distance = poseDataFuel.getFloat();
                                fvd.angle = poseDataFuel.getFloat();
                                fvd.orientation = poseDataFuel.get();
                                fvd.amount = poseDataFuel.getInt();
                                fuelArray[i] = fvd;
                            }

                            // save data to capture on next inputs loop
                            fuelHeader = fvh;
                            fuelData = fuelArray;
                        });
    }

    @Override
    public void updateInputs(FuelVisionIOInputs inputs) {
        FuelVisionHeader fvh = fuelHeader;
        inputs.seqNum = fvh.seqNum;
        inputs.rioTime = fvh.rioTime;
        inputs.imageTime = fvh.imageTime;
        inputs.realTime = fvh.realTime;

        inputs.fuelData = fuelData;

        inputs.now = RobotController.getFPGATime() / 1000000.0;
    }

    @Override
    public void fuelCamEnable(boolean enable) {
        fuelActive.set(enable);
    }
}
