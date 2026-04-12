package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class LedSubsystem extends SubsystemBase {

  private static final int GREEN_R = 0;
  private static final int GREEN_G = 255;
  private static final int GREEN_B = 0;
  private static final int YELLOW_R = 255;
  private static final int YELLOW_G = 255;
  private static final int YELLOW_B = 0;
  private static final int RED_R = 255;
  private static final int RED_G = 0;
  private static final int RED_B = 0;
  private static final int BLUE_R = 0;
  private static final int BLUE_G = 0;
  private static final int BLUE_B = 255;
  private static final RGBWColor GREEN = new RGBWColor(GREEN_R, GREEN_G, GREEN_B);
  private static final RGBWColor YELLOW = new RGBWColor(YELLOW_R, YELLOW_G, YELLOW_B);
  private static final RGBWColor RED = new RGBWColor(RED_R, RED_G, RED_B);
  private static final RGBWColor BLUE = new RGBWColor(BLUE_R, BLUE_G, BLUE_B);
  private static final RGBWColor EXTERNAL_RED = GREEN;
  private static final RGBWColor EXTERNAL_GREEN = RED;
  private static final RGBWColor EXTERNAL_YELLOW = YELLOW;
  private static final RGBWColor EXTERNAL_BLUE = BLUE;
  private static final double PRE_SHIFT_WARNING_SECONDS = 5.0;
  private static final double FLASH_HZ = 3.0;

  CANdle candle = new CANdle(LEDConstants.candleID);
  private final SolidColor onboardSolidRequest =
      new SolidColor(LEDConstants.onboardStartIdx, LEDConstants.onboardEndIdx);
  private final SolidColor externalSolidRequest =
      new SolidColor(LEDConstants.kSlot1StartIdx, LEDConstants.kSlot1EndIdx);
  private final RainbowAnimation rainbowRequest =
      new RainbowAnimation(LEDConstants.onboardStartIdx, LEDConstants.kSlot1EndIdx).withSlot(0);

  private enum LedState {
    RAINBOW,
    SOLID_RED,
    SOLID_BLUE,
    SOLID_GREEN,
    SOLID_YELLOW
  }

  private LedState hardwareState = LedState.SOLID_YELLOW;
  private Optional<Alliance> latchedAlliance = Optional.empty();
  private String latchedGameData = "";

  public LedSubsystem() {

    var config = new CANdleConfiguration();
    /* set the LED strip type and brightness */
    config.LED.StripType = StripTypeValue.GRB;
    config.LED.BrightnessScalar = 1;
    /* disable status LED when being controlled */
    config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

    candle.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    updateLatching();

    if (DriverStation.isDisabled()) {
      if (!DriverStation.isDSAttached()) {
        setLedStateWithReason(LedState.RAINBOW, "DISABLED", "NO_DS");
        return;
      }

      Optional<LedState> allianceColor = getDriverStationAllianceState();
      if (allianceColor.isPresent()) {
        setLedStateWithReason(allianceColor.get(), "DISABLED", "DS_ALLIANCE");
      } else {
        setLedStateWithReason(LedState.SOLID_YELLOW, "DISABLED", "FALLBACK_YELLOW");
      }
      return;
    }

    // ENABLED CASE
    double matchTime = DriverStation.getMatchTime();
    boolean hasNoTimer = matchTime <= 0;
    boolean hasNoFms = !DriverStation.isFMSAttached();

    LedState baseEnabledState;
    String decisionSource;

    Optional<Boolean> ourShiftActive = getOurAllianceShiftActive(matchTime);
    if (ourShiftActive.isPresent()) {
      baseEnabledState = ourShiftActive.get() ? LedState.SOLID_GREEN : LedState.SOLID_YELLOW;
      decisionSource = "SHIFT_ACTIVE";
    } else if (hasNoTimer && hasNoFms) {
      Optional<LedState> gameDataColor = getGameDataColorState();
      if (gameDataColor.isPresent()) {
        baseEnabledState = gameDataColor.get();
        decisionSource = "GAME_DATA";
      } else {
        Optional<LedState> allianceColor = getDriverStationAllianceState();
        if (allianceColor.isPresent()) {
          baseEnabledState = allianceColor.get();
          decisionSource = "DS_ALLIANCE";
        } else {
          baseEnabledState = LedState.SOLID_YELLOW;
          decisionSource = "FALLBACK_YELLOW";
        }
      }
    } else {
      Optional<LedState> allianceColor = getDriverStationAllianceState();
      if (allianceColor.isPresent()) {
        baseEnabledState = allianceColor.get();
        decisionSource = "DS_ALLIANCE";
      } else {
        baseEnabledState = LedState.SOLID_YELLOW;
        decisionSource = "FALLBACK_YELLOW";
      }
    }

    if (isPreShiftWarningWindow(matchTime) && isYellowFlashOn()) {
      setLedStateWithReason(LedState.SOLID_YELLOW, "ENABLED", "PRE_SHIFT_WARNING");
    } else {
      setLedStateWithReason(baseEnabledState, "ENABLED", decisionSource);
    }
  }

  private void updateLatching() {
    Optional<Alliance> currentAlliance = DriverStation.getAlliance();
    if (currentAlliance.isPresent()) {
      latchedAlliance = currentAlliance;
    }

    String currentGameData = DriverStation.getGameSpecificMessage();
    if (!currentGameData.isEmpty()) {
      latchedGameData = currentGameData.trim().toUpperCase();
    }
  }

  private Optional<Character> getAllianceCharFromGameData() {
    String currentGameData = DriverStation.getGameSpecificMessage();
    if (!currentGameData.isEmpty()) {
      char currentChar = Character.toUpperCase(currentGameData.trim().charAt(0));
      if (currentChar == 'R' || currentChar == 'B') {
        return Optional.of(currentChar);
      }
    }

    if (!latchedGameData.isEmpty()) {
      char latchedChar = Character.toUpperCase(latchedGameData.charAt(0));
      if (latchedChar == 'R' || latchedChar == 'B') {
        return Optional.of(latchedChar);
      }
    }

    return Optional.empty();
  }

  private Optional<LedState> getDriverStationAllianceState() {
    Optional<Alliance> currentAlliance = DriverStation.getAlliance();
    if (currentAlliance.isPresent()) {
      return Optional.of(
          currentAlliance.get() == Alliance.Red ? LedState.SOLID_RED : LedState.SOLID_BLUE);
    }

    if (latchedAlliance.isPresent()) {
      return Optional.of(
          latchedAlliance.get() == Alliance.Red ? LedState.SOLID_RED : LedState.SOLID_BLUE);
    }

    return Optional.empty();
  }

  private Optional<LedState> getGameDataColorState() {
    Optional<Character> gameDataAlliance = getAllianceCharFromGameData();
    if (gameDataAlliance.isPresent()) {
      return Optional.of(gameDataAlliance.get() == 'R' ? LedState.SOLID_RED : LedState.SOLID_BLUE);
    }

    return Optional.empty();
  }

  private boolean isPreShiftWarningWindow(double matchTime) {
    if (!DriverStation.isTeleopEnabled() || matchTime <= 0) {
      return false;
    }

    return isWithinWindow(matchTime, 105.0 + PRE_SHIFT_WARNING_SECONDS, 105.0)
        || isWithinWindow(matchTime, 80.0 + PRE_SHIFT_WARNING_SECONDS, 80.0)
        || isWithinWindow(matchTime, 55.0 + PRE_SHIFT_WARNING_SECONDS, 55.0)
        || isWithinWindow(matchTime, 30.0 + PRE_SHIFT_WARNING_SECONDS, 30.0);
  }

  private boolean isWithinWindow(double value, double maxInclusive, double minExclusive) {
    return value <= maxInclusive && value > minExclusive;
  }

  private boolean isYellowFlashOn() {
    return ((int) (Timer.getTimestamp() * FLASH_HZ * 2.0) % 2) == 0;
  }

  private void setLedState(LedState newState) {
    if (hardwareState == newState) {
      return;
    }
    hardwareState = newState;

    switch (newState) {
      case RAINBOW -> candle.setControl(rainbowRequest);
      case SOLID_RED -> applySplitSolidColor(RED, EXTERNAL_RED);
      case SOLID_BLUE -> applySplitSolidColor(BLUE, EXTERNAL_BLUE);
      case SOLID_GREEN -> applySplitSolidColor(GREEN, EXTERNAL_GREEN);
      case SOLID_YELLOW -> applySplitSolidColor(YELLOW, EXTERNAL_YELLOW);
    }
  }

  private void applySplitSolidColor(RGBWColor onboardColor, RGBWColor externalColor) {
    candle.setControl(onboardSolidRequest.withColor(onboardColor));
    candle.setControl(externalSolidRequest.withColor(externalColor));
  }

  private void setLedStateWithReason(LedState newState, String mode, String source) {
    setLedState(newState);
    Logger.recordOutput("LED/Mode", mode);
    Logger.recordOutput("LED/DecisionSource", source);
    Logger.recordOutput("LED/RequestedState", newState.name());
    Logger.recordOutput("LED/AppliedState", hardwareState.name());
    Logger.recordOutput("LED/DSAttached", DriverStation.isDSAttached());
    Logger.recordOutput("LED/FMSAttached", DriverStation.isFMSAttached());
    Logger.recordOutput("LED/Disabled", DriverStation.isDisabled());
    Logger.recordOutput("LED/TeleopEnabled", DriverStation.isTeleopEnabled());
    Logger.recordOutput("LED/GameDataRaw", DriverStation.getGameSpecificMessage());
    Logger.recordOutput("LED/GameDataLatched", latchedGameData);
    Logger.recordOutput("LED/MatchTime", DriverStation.getMatchTime());
    Logger.recordOutput("LED/HasLatchedAlliance", latchedAlliance.isPresent());
    Logger.recordOutput(
        "LED/LatchedAlliance",
        latchedAlliance.isPresent() ? latchedAlliance.get().name() : "UNKNOWN");
  }

  private Optional<Boolean> getOurAllianceShiftActive(double matchTime) {
    if (latchedAlliance.isEmpty()) {
      return Optional.empty();
    }

    if (!DriverStation.isTeleopEnabled()) {
      return Optional.empty();
    }

    Optional<Character> gameDataAlliance = getAllianceCharFromGameData();
    if (gameDataAlliance.isEmpty()) {
      return Optional.empty();
    }

    boolean redInactiveFirst;
    switch (gameDataAlliance.get()) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        return Optional.empty();
      }
    }

    // Shift 1 is active for blue if red is inactive first, or red if blue is inactive first.
    boolean shift1Active =
        switch (latchedAlliance.get()) {
          case Red -> !redInactiveFirst;
          case Blue -> redInactiveFirst;
        };

    if (matchTime > 105) {
      // Initial 5s transition at teleop start.
      return Optional.of(true);
    } else if (matchTime > 80) {
      return Optional.of(shift1Active);
    } else if (matchTime > 55) {
      return Optional.of(!shift1Active);
    } else if (matchTime > 30) {
      return Optional.of(shift1Active);
    } else {
      // End game and after: no alliance shift requirement.
      return Optional.of(false);
    }
  }
}
