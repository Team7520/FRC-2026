package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LedSubsystem extends SubsystemBase {

  CANdle candle = new CANdle(LEDConstants.candleID);

  public LedSubsystem() {

    var config = new CANdleConfiguration();
    /* set the LED strip type and brightness */
    // config.LED.StripType = StripTypeValue.GRB;
    config.LED.BrightnessScalar = 1;
    /* disable status LED when being controlled */
    config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

    candle.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    candle.setControl(new RainbowAnimation(LEDConstants.onboardStartIdx, 46).withSlot(0));
  }
}
