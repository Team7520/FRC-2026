package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/** Operator adjustments applied to each newly calculated shot. Resets when robot code restarts. */
public class ShotTuning {
  public static final double POWER_STEP_RPS = 0.5;
  public static final double AIM_STEP_DEGREES = 0.5;

  private double powerOffsetRps = 0.0;
  private double aimOffsetDegrees = 0.0;

  public void increasePower() {
    powerOffsetRps += POWER_STEP_RPS;
  }

  public void decreasePower() {
    powerOffsetRps -= POWER_STEP_RPS;
  }

  public void aimLeft() {
    // Positive matches the existing leftward manual turret command.
    aimOffsetDegrees += AIM_STEP_DEGREES;
  }

  public void aimRight() {
    aimOffsetDegrees -= AIM_STEP_DEGREES;
  }

  public double applyPower(double baseRps, double maxRps) {
    double baseTarget = MathUtil.clamp(baseRps, 0.0, maxRps);
    return MathUtil.clamp(baseTarget + powerOffsetRps, 0.0, maxRps);
  }

  public Rotation2d applyAim(Rotation2d baseAngle) {
    return baseAngle.plus(Rotation2d.fromDegrees(aimOffsetDegrees));
  }

  public double getPowerOffsetRps() {
    return powerOffsetRps;
  }

  public double getAimOffsetDegrees() {
    return aimOffsetDegrees;
  }
}
