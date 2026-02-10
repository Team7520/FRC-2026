package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

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

  public TurretSubsystem(int turnMotorId, int hoodMotorId, int topMotorId, int bottomMotorId) {
    SmartMotorControllerConfig turretConfig =
        new SmartMotorControllerConfig(this)
            .withClosedLoopController(
                4.0, 0.0, 0.0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
            .withSoftLimit(TURRET_MIN_ANGLE, TURRET_MAX_ANGLE)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(40))
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
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withStartingPosition(Degrees.of(0));

    SmartMotorControllerConfig shooterConfig =
        new SmartMotorControllerConfig(this)
            .withStatorCurrentLimit(Amps.of(40))
            .withOpenLoopRampRate(OPEN_LOOP_RAMP)
            .withIdleMode(MotorMode.COAST)
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

  public void turn(double speed) {
    turretMotor.setDutyCycle(speed);
  }

  public void hood(double speed) {
    hoodMotor.setDutyCycle(speed);
  }

  public void top(double speed) {
    topMotor.setDutyCycle(speed);
  }

  public void bottom(double speed) {
    bottomMotor.setDutyCycle(speed);
  }

  public void stopAll() {
    turretMotor.setDutyCycle(0.0);
    hoodMotor.setDutyCycle(0.0);
    topMotor.setDutyCycle(0.0);
    bottomMotor.setDutyCycle(0.0);
  }
}
