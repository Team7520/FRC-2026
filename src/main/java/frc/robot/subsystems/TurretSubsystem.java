package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class TurretSubsystem extends SubsystemBase {
  private static final Time OPEN_LOOP_RAMP = Seconds.of(0.15);

  private static final Angle TURRET_MIN_ANGLE = Degrees.of(-180);
  private static final Angle TURRET_MAX_ANGLE = Degrees.of(180);
  private static final Angle HOOD_MIN_ANGLE = Degrees.of(-10);
  private static final Angle HOOD_MAX_ANGLE = Degrees.of(45);

  private final TalonFXWrapper turretMotor;
  private final Pivot turret;
  private final TalonFXWrapper hoodMotor;
  private final Pivot hood;
  private final TalonFXWrapper topMotor;
  private final TalonFXWrapper bottomMotor;
  private final TalonFXWrapper feedMotor;

  public TurretSubsystem(
      int turnMotorId, int hoodMotorId, int topMotorId, int bottomMotorId, int feederMotorId) {
    TalonFXConfiguration phoenixProConfig = createPhoenixProConfig();

    SmartMotorControllerConfig turretConfig =
        new SmartMotorControllerConfig(this)
            .withClosedLoopController(
                4.0, 0.0, 0.0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
            .withSoftLimit(TURRET_MIN_ANGLE, TURRET_MAX_ANGLE)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(40))
            .withVendorConfig(phoenixProConfig)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withStartingPosition(Degrees.of(0));

    SmartMotorControllerConfig hoodConfig =
        new SmartMotorControllerConfig(this)
            .withClosedLoopController(
                4.0, 0.0, 0.0, DegreesPerSecond.of(120), DegreesPerSecondPerSecond.of(60))
            .withSoftLimit(HOOD_MIN_ANGLE, HOOD_MAX_ANGLE)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(30))
            .withVendorConfig(phoenixProConfig)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withStartingPosition(Degrees.of(0));

    SmartMotorControllerConfig shooterConfig =
        new SmartMotorControllerConfig(this)
            .withStatorCurrentLimit(Amps.of(40))
            .withOpenLoopRampRate(OPEN_LOOP_RAMP)
            .withIdleMode(MotorMode.COAST)
            .withVendorConfig(phoenixProConfig)
            .withControlMode(ControlMode.OPEN_LOOP);

    SmartMotorControllerConfig feederConfig =
        new SmartMotorControllerConfig(this)
            .withStatorCurrentLimit(Amps.of(40))
            .withOpenLoopRampRate(OPEN_LOOP_RAMP)
            .withIdleMode(MotorMode.BRAKE)
            .withVendorConfig(phoenixProConfig)
            .withControlMode(ControlMode.OPEN_LOOP);

    turretMotor =
        new TalonFXWrapper(
            new TalonFX(turnMotorId, TunerConstants.kCANBus),
            DCMotor.getKrakenX60(1),
            turretConfig);
    hoodMotor =
        new TalonFXWrapper(
            new TalonFX(hoodMotorId, TunerConstants.kCANBus), DCMotor.getKrakenX60(1), hoodConfig);
    topMotor =
        new TalonFXWrapper(
            new TalonFX(topMotorId, TunerConstants.kCANBus),
            DCMotor.getKrakenX60(1),
            shooterConfig);
    bottomMotor =
        new TalonFXWrapper(
            new TalonFX(bottomMotorId, TunerConstants.kCANBus),
            DCMotor.getKrakenX60(1),
            shooterConfig);
    feedMotor =
        new TalonFXWrapper(
            new TalonFX(feederMotorId, TunerConstants.kCANBus),
            DCMotor.getKrakenX60(1),
            feederConfig);

    turret =
        new Pivot(
            new PivotConfig(turretMotor)
                .withHardLimit(TURRET_MIN_ANGLE, TURRET_MAX_ANGLE)
                .withStartingPosition(Degrees.of(0)));
    hood =
        new Pivot(
            new PivotConfig(hoodMotor)
                .withHardLimit(HOOD_MIN_ANGLE, HOOD_MAX_ANGLE)
                .withStartingPosition(Degrees.of(0)));
  }

  public void setTurretAngle(Angle angle) {
    turretMotor.setPosition(angle);
  }

  public void setHoodAngle(Angle angle) {
    hoodMotor.setPosition(angle);
  }

  public Angle getTurretAngle() {
    return turret.getAngle();
  }

  public Angle getHoodAngle() {
    return hood.getAngle();
  }

  /**
   * Controls turret angle using a normalized speed value.
   * Maps speed [-1, 1] to the full turret angle range.
   *
   * @param speed Normalized position, where -1 maps to TURRET_MIN_ANGLE and 1 to TURRET_MAX_ANGLE
   */
  public void turn(double speed) {
    // Interpret speed in [-1, 1] as a normalized position across the turret's angle range
    double clampedSpeed = Math.max(-1.0, Math.min(1.0, speed));
    // Map -1 -> TURRET_MIN_ANGLE, 0 -> midpoint, 1 -> TURRET_MAX_ANGLE
    double fraction = (clampedSpeed + 1.0) / 2.0;
    Angle targetAngle =
        TURRET_MIN_ANGLE.plus(TURRET_MAX_ANGLE.minus(TURRET_MIN_ANGLE).times(fraction));
    turretMotor.setPosition(targetAngle);
  }

  public void turnToPosition(double turretPosition) {
    turretMotor.setPosition(Rotations.of(turretPosition));
  }

  public void hoodToPosition(double hoodPosition) {
    hoodMotor.setPosition(Rotations.of(hoodPosition));
  }

  /**
   * Controls hood angle using a normalized speed value.
   * Maps speed [-1, 1] to the full hood angle range.
   *
   * @param speed Normalized position, where -1 maps to HOOD_MIN_ANGLE and 1 to HOOD_MAX_ANGLE
   */
  public void hood(double speed) {
    // Interpret speed in [-1, 1] as a normalized position across the hood's angle range
    double clampedSpeed = Math.max(-1.0, Math.min(1.0, speed));
    // Map -1 -> HOOD_MIN_ANGLE, 0 -> midpoint, 1 -> HOOD_MAX_ANGLE
    double fraction = (clampedSpeed + 1.0) / 2.0;
    Angle targetAngle = HOOD_MIN_ANGLE.plus(HOOD_MAX_ANGLE.minus(HOOD_MIN_ANGLE).times(fraction));
    hoodMotor.setPosition(targetAngle);
  }

  public void top(double speed) {
    topMotor.setDutyCycle(speed);
  }

  public void bottom(double speed) {
    bottomMotor.setDutyCycle(speed);
  }

  public void feeder(double speed) {
    feedMotor.setDutyCycle(speed);
  }

  public void stopAll() {
    // Turret and hood hold their current closed-loop positions.
    // Only stop the shooter wheels and feeder, which are driven open-loop.
    topMotor.setDutyCycle(0.0);
    bottomMotor.setDutyCycle(0.0);
    feedMotor.setDutyCycle(0.0);
  }
}
