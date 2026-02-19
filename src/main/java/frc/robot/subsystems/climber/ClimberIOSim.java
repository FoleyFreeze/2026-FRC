package frc.robot.subsystems.climber;

public class ClimberIOSim implements ClimberIO {
    public boolean climb1Connected = false;
    public double climb1Voltage = 0;
    public double climb1Current = 0;
    public double climb1Position = 0;
    public double climb1Velocity = 0;

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climb1Connected = true;
        inputs.climb1Position = climb1Position;
    }

    @Override
    public void setPower1(double power) {
        climb1Voltage = (power * 12);
        climb1Position =
                climb1Voltage * (90 / 12); // power is 0-1, same as volts of 0-12, which is then the
        // same as 0-90 in terms of angle
    }
}
