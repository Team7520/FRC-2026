package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import java.util.Optional;

public class LightingSubsystem extends SubsystemBase {
  // Note: CANdle ID 17 is used on the CANivore bus. Verify this ID doesn't conflict
  // with existing swerve drive devices (motors, CANcoders, Pigeon 2). Consider moving
  // to RIO CAN 2.0 bus if conflicts occur.
  private static final int CANDLE_ID = 17;
  private static final int CANDLE_LED_COUNT = 32;
  private static final int SIDE_LED_COUNT = 40;
  private static final int SIDE_LED_PWM_PORT = 0;
  private static final double CANDLE_BRIGHTNESS = 0.75;

  private static final Distance SIDE_LED_SPACING = Meters.of(1 / 60.0);

  private final CANdle candle = new CANdle(CANDLE_ID, TunerConstants.kCANBus);
  private final AddressableLED sideLED = new AddressableLED(SIDE_LED_PWM_PORT);
  private final AddressableLEDBuffer sideLEDBuffer = new AddressableLEDBuffer(SIDE_LED_COUNT);

  public LightingSubsystem() {
    sideLED.setLength(sideLEDBuffer.getLength());
    sideLED.setData(sideLEDBuffer);
    sideLED.start();

    CANdleConfiguration candleConfig = new CANdleConfiguration();
    LEDConfigs ledConfigs = new LEDConfigs();
    ledConfigs.StripType = StripTypeValue.GRB;
    ledConfigs.BrightnessScalar = CANDLE_BRIGHTNESS;
    candleConfig.LED = ledConfigs;
    candle.getConfigurator().apply(candleConfig);

    setColor(127, 127, 127);
  }

  public void setColor(int red, int green, int blue) {
    candle.setControl(
        new SolidColor(0, CANDLE_LED_COUNT - 1).withColor(new RGBWColor(red, green, blue)));
    setSideLEDs(red, green, blue);
  }

  public void colorFlow(int red, int green, int blue) {
    ColorFlowAnimation animation =
        new ColorFlowAnimation(0, CANDLE_LED_COUNT - 1)
            .withColor(new RGBWColor(red, green, blue))
            .withDirection(AnimationDirectionValue.Forward)
            .withFrameRate(0.5);
    candle.setControl(animation);
    setSideLEDs(red, green, blue);
  }

  public void rainbow() {
    RainbowAnimation animation =
        new RainbowAnimation(0, CANDLE_LED_COUNT - 1).withBrightness(1.0).withFrameRate(0.5);
    candle.setControl(animation);
    LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    LEDPattern scrollingRainbow =
        rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), SIDE_LED_SPACING);
    scrollingRainbow.applyTo(sideLEDBuffer);
    sideLED.setData(sideLEDBuffer);
  }

  public void allianceColor() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        setColor(255, 0, 0);
        return;
      }
      if (ally.get() == Alliance.Blue) {
        setColor(0, 0, 255);
        return;
      }
    }
    rainbow();
  }

  private void setSideLEDs(int red, int green, int blue) {
    for (int i = 0; i < sideLEDBuffer.getLength(); i++) {
      sideLEDBuffer.setRGB(i, red, green, blue);
    }
    sideLED.setData(sideLEDBuffer);
  }
}
