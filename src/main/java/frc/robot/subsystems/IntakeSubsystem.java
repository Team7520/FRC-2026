package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakeSubsystem extends SubsystemBase {
  private static final boolean FORCE_PHOENIX_PRO = true;

  private static final int INTAKE_MOTOR_ID = 1;
  private static final int EXTEND_MOTOR_ID = 2; // placeholder ids
  private static final int INDEXER_MOTOR_ID = 3;

  private static final Time OPEN_LOOP_RAMP = Seconds.of(0.15);
  private static final Angle EXTENDED_POSITION = Degrees.of(90);
  private static final Angle RETRACTED_POSITION = Degrees.of(0);

  private final TalonFXWrapper intakeMotor;
  private final TalonFXWrapper extendMotor;
  private final TalonFXWrapper indexerMotor;
  private final TorqueCurrentFOC openLoopRequest = new TorqueCurrentFOC(0.0);
  private final PositionTorqueCurrentFOC positionRequest = new PositionTorqueCurrentFOC(0.0);

  private Angle extendedPosition = EXTENDED_POSITION;
  private Angle retractedPosition = RETRACTED_POSITION;

  public IntakeSubsystem() {
    TalonFXConfiguration phoenixProConfig = createPhoenixProConfig();
    SmartMotorControllerConfig intakeConfig =
        new SmartMotorControllerConfig(this)
            .withStatorCurrentLimit(Amps.of(40))
            .withOpenLoopRampRate(OPEN_LOOP_RAMP)
            .withIdleMode(MotorMode.COAST)
            .withVendorConfig(phoenixProConfig)
            .withControlMode(ControlMode.OPEN_LOOP);

    SmartMotorControllerConfig extendConfig =
        new SmartMotorControllerConfig(this)
            .withClosedLoopController(
                4.0, 0.0, 0.0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
            .withSoftLimit(Degrees.of(-30), Degrees.of(100))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(40))
            .withVendorConfig(phoenixProConfig)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withStartingPosition(retractedPosition);

    intakeMotor =
        new TalonFXWrapper(
            new TalonFX(INTAKE_MOTOR_ID, TunerConstants.kCANBus),
            DCMotor.getKrakenX60(1),
            intakeConfig);
    extendMotor =
        new TalonFXWrapper(
            new TalonFX(EXTEND_MOTOR_ID, TunerConstants.kCANBus),
            DCMotor.getKrakenX60(1),
            extendConfig);
    indexerMotor =
        new TalonFXWrapper(
            new TalonFX(INDEXER_MOTOR_ID, TunerConstants.kCANBus),
            DCMotor.getKrakenX60(1),
            intakeConfig);
  }

  public void runIntake(double speed) {
    if (FORCE_PHOENIX_PRO) {
      getMotorController(intakeMotor).setControl(openLoopRequest.withOutput(speed));
      return;
    }
    intakeMotor.setDutyCycle(speed);
  }

  public void extend() {
    if (FORCE_PHOENIX_PRO) {
      getMotorController(extendMotor)
          .setControl(positionRequest.withPosition(extendedPosition.in(Rotations)));
      return;
    }
    extendMotor.setPosition(extendedPosition);
  }

  public void retract() {
    if (FORCE_PHOENIX_PRO) {
      getMotorController(extendMotor)
          .setControl(positionRequest.withPosition(retractedPosition.in(Rotations)));
      return;
    }
    extendMotor.setPosition(retractedPosition);
  }

  public void runIndexer(double speed) {
    if (FORCE_PHOENIX_PRO) {
      getMotorController(indexerMotor).setControl(openLoopRequest.withOutput(speed));
      return;
    }
    indexerMotor.setDutyCycle(speed);
  }

  public void setExtendedPosition(Angle position) {
    extendedPosition = position;
  }

  public void setRetractedPosition(Angle position) {
    retractedPosition = position;
  }

  public void stopAll() {
    if (FORCE_PHOENIX_PRO) {
      getMotorController(intakeMotor).setControl(openLoopRequest.withOutput(0.0));
      getMotorController(extendMotor).setControl(openLoopRequest.withOutput(0.0));
      getMotorController(indexerMotor).setControl(openLoopRequest.withOutput(0.0));
      return;
    }
    intakeMotor.setDutyCycle(0.0);
    extendMotor.setDutyCycle(0.0);
    indexerMotor.setDutyCycle(0.0);
  }

  private TalonFX getMotorController(TalonFXWrapper wrapper) {
    return (TalonFX) wrapper.getMotorController();
  }

  private TalonFXConfiguration createPhoenixProConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotionMagic.MotionMagicExpo_kV = 0.12;
    config.MotionMagic.MotionMagicExpo_kA = 0.1;
    return config;
  }
}
