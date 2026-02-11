package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class TurretSubsystem extends SubsystemBase {
  private final TalonFX turnMotor;
  private final TalonFX hoodMotor;
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final TalonFX feedMotor;
  private final DutyCycleOut duty = new DutyCycleOut(0);
  private final PositionDutyCycle pivotPosReq = new PositionDutyCycle(0);
  private final PositionDutyCycle anglePosReq = new PositionDutyCycle(0);

  public TurretSubsystem(int turnMotorId, int hoodMotorId, int topMotorId, int bottomMotorId, int feederMotorId) {
    turnMotor = new TalonFX(turnMotorId);
    hoodMotor = new TalonFX(hoodMotorId);
    leftMotor = new TalonFX(topMotorId);
    rightMotor = new TalonFX(bottomMotorId);
    feedMotor = new TalonFX(feederMotorId);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80.0;
    
    turnMotor.getConfigurator().apply(config);
    turnMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
    hoodMotor.getConfigurator().apply(config);
    hoodMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
    leftMotor.getConfigurator().apply(config);
    leftMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
    rightMotor.getConfigurator().apply(config);
    rightMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
    feedMotor.getConfigurator().apply(config);
    feedMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
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

  public void top(double speed) {
    leftMotor.setControl(duty.withOutput(speed));
  }

  public void bottom(double speed) {
    rightMotor.setControl(duty.withOutput(speed));
  }

  public void feeder(double speed) {
    feedMotor.setControl(duty.withOutput(speed));
  }

  public void stopAll() {
    turnMotor.setControl(duty.withOutput(0));
    hoodMotor.setControl(duty.withOutput(0));
    leftMotor.setControl(duty.withOutput(0));
    rightMotor.setControl(duty.withOutput(0));
    feedMotor.setControl(duty.withOutput(0));
  }
}
