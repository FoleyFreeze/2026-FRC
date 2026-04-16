package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Led extends SubsystemBase {
    AddressableLED leds;
    private RobotContainer r;
    AddressableLEDBuffer buffer;

    public enum LED_MODES {
        OFF(LEDPattern.solid(Color.kBlack)),
        BLUE(LEDPattern.solid(Color.kBlue)),
        GREEN(LEDPattern.solid(Color.kGreen)),
        BLINK_GREEN(LEDPattern.solid(Color.kGreen).blink(Seconds.of(0.4), Seconds.of(0.1))),
        RED(LEDPattern.solid(Color.kRed)),
        BLINK_BLUE(LEDPattern.solid(Color.kBlue).blink(Seconds.of(1), Seconds.of(1))),
        BREATHE_BLUE(LEDPattern.solid(Color.kBlue).breathe(Seconds.of(3))),
        WHITE(LEDPattern.solid(Color.kWhite).atBrightness(Percent.of(100))),
        YELLOW(LEDPattern.solid(Color.kYellow).atBrightness(Percent.of(100))),

        RAINBOW(LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Seconds.of(5).asFrequency()));

        public final LEDPattern pattern;

        private LED_MODES(LEDPattern pattern) {
            this.pattern = pattern;
        }
    }

    public Led(RobotContainer r) {
        this.r = r;
        leds = new AddressableLED(9);
        // leds.setSyncTime(280);
        // leds.setBitTiming(0,0,0,0);
        leds.setColorOrder(ColorOrder.kRGB);
        buffer = new AddressableLEDBuffer(49);

        leds.setLength(buffer.getLength());
        leds.start();
    }

    @Override
    public void periodic() {
        leds.setData(buffer);
    }

    public Command setLEDMode(LED_MODES mode) {
        Command c = new RunCommand(() -> mode.pattern.applyTo(buffer), this).ignoringDisable(true);
        return c;
    }
}
