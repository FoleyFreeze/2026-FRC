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
import frc.robot.subsystems.shooter.Shooter.MissReason;
import org.littletonrobotics.junction.Logger;

public class Led extends SubsystemBase {
    AddressableLED leds;
    private RobotContainer r;
    AddressableLEDBuffer buffer;
    AddressableLEDBufferView front;
    AddressableLEDBufferView tip;
    AddressableLEDBufferView back;

    public static boolean isShoot;
    public static boolean isGather;

    public enum LED_MODES {
        OFF(LEDPattern.solid(Color.kBlack)),
        BLUE(LEDPattern.solid(Color.kBlue)),
        GREEN(LEDPattern.solid(Color.kGreen)),
        BLINK_GREEN(LEDPattern.solid(Color.kGreen).blink(Seconds.of(0.4), Seconds.of(0.1))),
        RED(LEDPattern.solid(Color.kRed)),
        BLINK_BLUE(LEDPattern.solid(Color.kBlue).blink(Seconds.of(0.4), Seconds.of(0.1))),
        BLINK_RED(LEDPattern.solid(Color.kRed).blink(Seconds.of(0.4), Seconds.of(0.1))),
        BREATHE_BLUE(LEDPattern.solid(Color.kBlue).breathe(Seconds.of(3))),
        WHITE(LEDPattern.solid(Color.kWhite).atBrightness(Percent.of(100))),
        YELLOW(LEDPattern.solid(Color.kYellow).atBrightness(Percent.of(100))),
        // alternating green/blue blink
        BLINK_GREEN_BLUE(
                LEDPattern.solid(Color.kBlue)
                        .blink(Seconds.of(0.3), Seconds.of(0.3))
                        .overlayOn(LEDPattern.solid(Color.kGreen))),
        BLINK_GREEN_RED(
                LEDPattern.solid(Color.kRed)
                        .blink(Seconds.of(0.3), Seconds.of(0.3))
                        .overlayOn(LEDPattern.solid(Color.kGreen))),

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

        front = buffer.createView(0, 24);
        tip = buffer.createView(25, 30);
        back = buffer.createView(31, 48);

        leds.setLength(buffer.getLength());
        leds.start();
    }

    @Override
    public void periodic() {
        double ballCount = 0;
        for (int i = 0; i < r.fuelVision.inputs.fuelData.length; i++) {
            ballCount += r.fuelVision.inputs.fuelData[i].amount;
        }
        if (ballCount > 2) {
            LED_MODES.YELLOW.pattern.atBrightness(Percent.of(ballCount / 10)).applyTo(tip);
        } else {
            LED_MODES.OFF.pattern.applyTo(tip);
        }

        leds.setData(buffer);
    }

    public Command setLEDMode(LED_MODES modeIn) {
        Command c =
                new RunCommand(
                                () -> {
                                    LED_MODES mode = modeIn;
                                    if (isGather && !isShoot) {
                                        mode = LED_MODES.BLINK_GREEN;
                                    } else if (isShoot && !isGather) {
                                        if (r.shooter.missReason == MissReason.NONE
                                                || r.shooter.missReason == MissReason.BAD_TIME) {
                                            mode = LED_MODES.BLINK_BLUE;
                                        } else {
                                            mode = LED_MODES.BLINK_RED;
                                        }
                                    } else if (isShoot && isGather) {
                                        if (r.shooter.missReason == MissReason.NONE
                                                || r.shooter.missReason == MissReason.BAD_TIME) {
                                            mode = LED_MODES.BLINK_GREEN_BLUE;
                                        } else {
                                            mode = LED_MODES.BLINK_GREEN_RED;
                                        }
                                    }

                                    Logger.recordOutput("Leds/Pattern", mode.toString());

                                    mode.pattern.applyTo(front);
                                    mode.pattern.applyTo(back);
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
