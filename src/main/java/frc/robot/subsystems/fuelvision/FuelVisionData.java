package frc.robot.subsystems.fuelvision;

import edu.wpi.first.util.struct.StructSerializable;

public class FuelVisionData implements StructSerializable {
    float distance;
    float angle;
    byte orientation;
    short amount;

    public static final FuelVisionDataStruct struct = new FuelVisionDataStruct();
}
