package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class ShotTuningTest {
  private static final double EPSILON = 1e-9;

  @Test
  void powerTrimPersistsAcrossRecalculatedShotsWithoutCompounding() {
    ShotTuning tuning = new ShotTuning();
    assertEquals(35.0, tuning.applyPower(35.0, 75.0), EPSILON);
    tuning.increasePower();
    tuning.increasePower();
    for (int cycle = 0; cycle < 100; cycle++) {
      assertEquals(36.0, tuning.applyPower(35.0, 75.0), EPSILON);
    }
    assertEquals(43.0, tuning.applyPower(42.0, 75.0), EPSILON);
    tuning.decreasePower();
    assertEquals(42.5, tuning.applyPower(42.0, 75.0), EPSILON);
    tuning.decreasePower();
    assertEquals(42.0, tuning.applyPower(42.0, 75.0), EPSILON);
  }

  @Test
  void powerTrimRespectsShootingAndPassingLimits() {
    ShotTuning tuning = new ShotTuning();
    for (int press = 0; press < 10; press++) {
      tuning.increasePower();
    }
    assertEquals(75.0, tuning.applyPower(74.0, 75.0), EPSILON);
    assertEquals(160.0, tuning.applyPower(159.0, 160.0), EPSILON);
    for (int press = 0; press < 30; press++) {
      tuning.decreasePower();
    }
    assertEquals(0.0, tuning.applyPower(5.0, 75.0), EPSILON);
    assertEquals(0.0, tuning.applyPower(5.0, 160.0), EPSILON);
  }

  @Test
  void decreasingPowerAlsoWorksWhenTheDistanceCurveExceedsTheSpeedLimit() {
    ShotTuning tuning = new ShotTuning();
    tuning.decreasePower();
    assertEquals(74.5, tuning.applyPower(90.0, 75.0), EPSILON);
    assertEquals(159.5, tuning.applyPower(180.0, 160.0), EPSILON);
  }

  @Test
  void aimTrimKeepsItsDirectionWhenTheTargetChangesOrCrossesTheAngleWrap() {
    ShotTuning tuning = new ShotTuning();
    tuning.aimLeft();
    assertEquals(0.5, tuning.applyAim(Rotation2d.kZero).getDegrees(), EPSILON);
    assertEquals(90.5, tuning.applyAim(Rotation2d.fromDegrees(90)).getDegrees(), EPSILON);
    assertEquals(-179.7, tuning.applyAim(Rotation2d.fromDegrees(179.8)).getDegrees(), EPSILON);
    tuning.aimRight();
    tuning.aimRight();
    assertEquals(-0.5, tuning.applyAim(Rotation2d.kZero).getDegrees(), EPSILON);
    assertEquals(179.7, tuning.applyAim(Rotation2d.fromDegrees(-179.8)).getDegrees(), EPSILON);
    tuning.aimLeft();
    assertEquals(0.0, tuning.getAimOffsetDegrees(), EPSILON);
  }
}
