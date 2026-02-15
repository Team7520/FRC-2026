package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
  private final TalonFX turnMotor;
  private final TalonFX hoodMotor;
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final TalonFX feedMotor;
  private final TalonFX indexMotor;
  private final DutyCycleOut duty = new DutyCycleOut(0);
  private final PositionDutyCycle pivotPosReq = new PositionDutyCycle(0);
  private final PositionDutyCycle anglePosReq = new PositionDutyCycle(0);

  public TurretSubsystem() {
    turnMotor = new TalonFX(TurretConstants.TURN_MOTOR, "CANivore");
    hoodMotor = new TalonFX(TurretConstants.HOOD_MOTOR, "CANivore");
    leftMotor = new TalonFX(TurretConstants.LEFT_MOTOR, "CANivore");
    rightMotor = new TalonFX(TurretConstants.RIGHT_MOTOR, "CANivore");
    feedMotor = new TalonFX(TurretConstants.FEEDER_MOTOR, "CANivore");
    indexMotor = new TalonFX(TurretConstants.INDEX_MOTOR, "CANivore");

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.CurrentLimits.StatorCurrentLimit = 20.0;
    config.Feedback.RotorToSensorRatio = 75;
    var limits = new SoftwareLimitSwitchConfigs();
    limits.ForwardSoftLimitEnable = true;
    limits.ForwardSoftLimitThreshold = 20.0 / 360.0;
    config.SoftwareLimitSwitch = limits;
    // turnMotor.getConfigurator().apply(config);
    // turnMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
    hoodMotor.getConfigurator().apply(config);
    hoodMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
    // leftMotor.getConfigurator().apply(config);
    // leftMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
    // rightMotor.getConfigurator().apply(config);
    // rightMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
    // feedMotor.getConfigurator().apply(config);
    // feedMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
  }

  public double getHoodPosition() {
    return hoodMotor.getPosition().getValueAsDouble();
  }

  public void turn(double speed) {
    turnMotor.setControl(duty.withOutput(speed));
  }

  public void turnToPosition(double turretPosition, double speed) {
    turnMotor.setControl(pivotPosReq.withPosition(turretPosition).withVelocity(speed));
  }

  public void turnToAngle(double hoodPosition, double speed) {
    hoodMotor.setControl(anglePosReq.withPosition(hoodPosition).withVelocity(speed));
  }

  public void hood(double speed) {
    hoodMotor.setControl(duty.withOutput(speed));
  }

  public void left(double speed) {
    leftMotor.setControl(duty.withOutput(speed));
  }

  public void right(double speed) {
    rightMotor.setControl(duty.withOutput(speed));
  }

  public void spinWheels(double speed) {
    rightMotor.setControl(duty.withOutput(-speed));
    leftMotor.setControl(duty.withOutput(speed));
  }

  public void feeder(double speed) {
    feedMotor.setControl(duty.withOutput(speed));
  }

  public void index(double speed) {
    indexMotor.setControl(duty.withOutput(speed));
  }

  public void stopAll() {
    turnMotor.setControl(duty.withOutput(0));
    hoodMotor.setControl(duty.withOutput(0));
    leftMotor.setControl(duty.withOutput(0));
    rightMotor.setControl(duty.withOutput(0));
    feedMotor.setControl(duty.withOutput(0));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("hoodMotor", hoodMotor.getPosition().getValueAsDouble());
  }
}
