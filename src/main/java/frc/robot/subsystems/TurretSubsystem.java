package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
  private final TalonFX turnMotor;
  private final TalonFX hoodMotor;
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final TalonFX feedMotor;
  private final CANcoder encoder;
  private final DutyCycleOut duty = new DutyCycleOut(0);
  private final PositionDutyCycle pivotPosReq = new PositionDutyCycle(0);
  private final PositionDutyCycle anglePosReq = new PositionDutyCycle(0);

  public TurretSubsystem() {
    turnMotor = new TalonFX(TurretConstants.TURN_MOTOR);
    hoodMotor = new TalonFX(TurretConstants.HOOD_MOTOR);
    leftMotor = new TalonFX(TurretConstants.LEFT_MOTOR);
    rightMotor = new TalonFX(TurretConstants.RIGHT_MOTOR);
    feedMotor = new TalonFX(TurretConstants.FEEDER_MOTOR);
    encoder = new CANcoder(53);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80.0;

    TalonFXConfiguration turnConfig = new TalonFXConfiguration();
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnConfig.CurrentLimits.StatorCurrentLimit = 30.0;
    turnConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turnConfig.Slot0.kP = 2;
    turnConfig.Slot0.kI = 0;
    turnConfig.Slot0.kD = 0;
    turnMotor.getConfigurator().apply(turnConfig);
    turnMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);

    leftMotor.getConfigurator().apply(config);
    leftMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
    rightMotor.getConfigurator().apply(config);
    rightMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
    feedMotor.getConfigurator().apply(config);
    feedMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);

    SoftwareLimitSwitchConfigs limits = new SoftwareLimitSwitchConfigs();
    limits.ForwardSoftLimitEnable = true;
    limits.ForwardSoftLimitThreshold = 6;
    limits.ReverseSoftLimitEnable = true;
    limits.ReverseSoftLimitThreshold = 0;

    config.SoftwareLimitSwitch = limits;
    hoodMotor.getConfigurator().apply(config);
    hoodMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
  }

  public Command moveToPosition(double position) {
    return Commands.run(() -> turnToPosition(position /*,  0.2*/), this)
        .until(() -> atTarget(position));
  }

  public boolean atTarget(double position) {
    double current = encoder.getAbsolutePosition().getValueAsDouble();
    double error = Math.abs(position - current);
    return error < 0.1;
  }

  public void turn(double speed) {
    turnMotor.setControl(duty.withOutput(speed));
  }

  public void turnToPosition(double turretPosition /*, double speed*/) {
    turnMotor.setControl(pivotPosReq.withPosition(turretPosition) /*.withVelocity(speed)*/);
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

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
        "Turret rotatations", encoder.getAbsolutePosition().getValueAsDouble());
  }
}
