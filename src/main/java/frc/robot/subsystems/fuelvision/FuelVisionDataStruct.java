package frc.robot.subsystems.fuelvision;

import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class FuelVisionDataStruct implements Struct<FuelVisionData> {

    @Override
    public Class<FuelVisionData> getTypeClass() {
        return FuelVisionData.class;
    }

    @Override
    public String getTypeString() {
        return "struct:FuelVisionData";
    }

    @Override
    public int getSize() {
        return kSizeFloat * 2 + 1 + kSizeInt16;
    }

    @Override
    public String getSchema() {
        return "float distance;float angle;byte orientation;short amount";
    }

    @Override
    public FuelVisionData unpack(ByteBuffer bb) {
        FuelVisionData v = new FuelVisionData();
        v.distance = bb.getFloat();
        v.angle = bb.getFloat();
        v.orientation = bb.get();
        v.amount = bb.getShort();
        return v;
    }

    @Override
    public void pack(ByteBuffer bb, FuelVisionData value) {
        bb.putFloat(value.distance);
        bb.putFloat(value.angle);
        bb.put(value.orientation);
        bb.putShort(value.amount);
    }

    @Override
    public String getTypeName() {
        return "FuelVisionData";
    }
}
