package frc.robot.subsystems.fuelvision;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.wpilibj.RobotController;
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
    }

    FuelVisionHeader fuelHeader;
    FuelVisionData[] fuelData;

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
                            fvh.seqNum = poseDataFuel.getInt();
                            fvh.rioTime = poseDataFuel.getFloat();
                            fvh.imageTime = poseDataFuel.getFloat();
                            int length = Byte.toUnsignedInt(poseDataFuel.get());

                            // parse fuel groups
                            FuelVisionData[] fuelArray = new FuelVisionData[length];
                            for (int i = 0; i < length; i++) {
                                FuelVisionData fvd = new FuelVisionData();
                                fvd.distance = poseDataFuel.getFloat();
                                fvd.angle = poseDataFuel.getFloat();
                                fvd.orientation = poseDataFuel.get();
                                fvd.amount = poseDataFuel.getShort();
                                fuelArray[i] = fvd;
                            }

                            // save data to capture on next inputs loop
                            fuelData = fuelArray;
                        });
    }

    @Override
    public void updateInputs(FuelVisionIOInputs inputs) {
        FuelVisionHeader fvh = fuelHeader;
        inputs.seqNum = fvh.seqNum;
        inputs.rioTime = fvh.rioTime;
        inputs.imageTime = fvh.imageTime;

        inputs.fuelData = fuelData;

        inputs.now = RobotController.getFPGATime() / 1000000.0;
    }

    @Override
    public void fuelCamEnable(boolean enable) {
        fuelActive.set(enable);
    }
}
