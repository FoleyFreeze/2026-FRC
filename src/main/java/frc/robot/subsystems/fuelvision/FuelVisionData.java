package frc.robot.subsystems.fuelvision;

import edu.wpi.first.util.struct.StructSerializable;

public class FuelVisionData implements StructSerializable {
    public float distance;
    public float angle;
    public byte orientation;
    public int amount;

    public static final FuelVisionDataStruct struct = new FuelVisionDataStruct();
}
