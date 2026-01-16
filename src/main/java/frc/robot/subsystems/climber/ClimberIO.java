package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public boolean climb1Connected = false;
        public double climb1Voltage = 0;
        public double climb1Current = 0;
        public double climb1Position = 0;
        public double climb1Velocity = 0;
        public double climb1Temp = 0;

        public boolean climb2Connected = false;
        public double climb2Voltage = 0;
        public double climb2Current = 0;
        public double climb2Position = 0;
        public double climb2Velocity = 0;
        public double climb2Temp = 0;
    }

    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setPos1(double position) {}

    public default void setPos2(double position) {}

    public default void setPower1(double power) {}

    public default void setPower2(double power) {}
}
