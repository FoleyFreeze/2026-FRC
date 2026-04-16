package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Led extends SubsystemBase {
    AddressableLED leds;
    private RobotContainer r;
    AddressableLEDBuffer buffer;
    AddressableLEDBufferView front;
    AddressableLEDBufferView tip;
    AddressableLEDBufferView back;

    private boolean isShoot;
    private boolean isGather;

    public enum LED_MODES {
        OFF(LEDPattern.solid(Color.kBlack)),
        BLUE(LEDPattern.solid(Color.kBlue)),
        GREEN(LEDPattern.solid(Color.kGreen)),
        BLINK_GREEN(LEDPattern.solid(Color.kGreen).blink(Seconds.of(0.4), Seconds.of(0.1))),
        RED(LEDPattern.solid(Color.kRed)),
        BLINK_BLUE(LEDPattern.solid(Color.kBlue).blink(Seconds.of(0.4), Seconds.of(0.1))),
        BREATHE_BLUE(LEDPattern.solid(Color.kBlue).breathe(Seconds.of(3))),
        WHITE(LEDPattern.solid(Color.kWhite).atBrightness(Percent.of(100))),
        YELLOW(LEDPattern.solid(Color.kYellow).atBrightness(Percent.of(100))),
        // BLINK_GREEN_BLUE(LEDPattern),

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

        front = buffer.createView(0, 22);
        tip = buffer.createView(23, 28);
        back = buffer.createView(29, 48);

        leds.setLength(buffer.getLength());
        leds.start();
    }

    @Override
    public void periodic() {
        leds.setData(buffer);

        double ballCount = 0;
        for (int i = 0; i < r.fuelVision.inputs.fuelData.length; i++) {
            ballCount += r.fuelVision.inputs.fuelData[i].amount;
        }
        if (ballCount > 2) {
            LED_MODES.YELLOW.pattern.atBrightness(Percent.of(ballCount / 10)).applyTo(tip);
        } else {
            LED_MODES.OFF.pattern.applyTo(tip);
        }
    }

    public Command setLEDMode(LED_MODES mode) {
        Command c =
                new RunCommand(
                                () -> {
                                    LEDPattern p = mode.pattern;

                                    if (isGather && !isShoot) {
                                        p = LED_MODES.GREEN.pattern;
                                    } else if (isShoot && !isGather) {
                                        p = LED_MODES.BLUE.pattern;
                                    } else if (isShoot && isGather) {
                                        p = LED_MODES.YELLOW.pattern;
                                    }

                                    p.applyTo(front);
                                    p.applyTo(back);
                                },
                                this)
                        .ignoringDisable(true);
        return c;
    }

    public Command setIsGather() {
        return new FunctionalCommand(
                () -> isGather = true, () -> {}, (b) -> isGather = false, () -> false);
    }

    public Command setIsShoot() {
        return new FunctionalCommand(
                () -> isShoot = true, () -> {}, (b) -> isShoot = false, () -> false);
    }
}
