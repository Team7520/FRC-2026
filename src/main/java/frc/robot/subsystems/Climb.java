package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climb extends SubsystemBase {
  private final TalonFX climberMotor;
  private double targetPosition;

  public Climb(int motorID) {
    climberMotor = new TalonFX(motorID);

    TalonFXConfiguration climberConfig = new TalonFXConfiguration();
    climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    climberConfig.CurrentLimits.SupplyCurrentLimit = ClimberConstants.MAX_AMP_POWER;
    climberConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    climberConfig.Voltage.PeakForwardVoltage = ClimberConstants.MAX_VOLTAGE;
    climberConfig.Voltage.PeakReverseVoltage = -ClimberConstants.MAX_VOLTAGE;
    climberConfig.Slot0.kP = ClimberConstants.KP;
    climberConfig.MotorOutput.PeakForwardDutyCycle = ClimberConstants.MAX_MOTOR_OUTPUT;
    climberConfig.MotorOutput.PeakReverseDutyCycle = -ClimberConstants.MAX_MOTOR_OUTPUT;

    climberConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    climberMotor.setPosition(0);

    climberMotor.getConfigurator().apply(climberConfig);
  }

  public void setPower(double power) {
    climberMotor.set(power);
    targetPosition = climberMotor.getPosition().getValueAsDouble();
  }

  public void holdPosition() {
    climberMotor.setPosition(targetPosition);
  }

  public void goToPosition(double position){
    targetPosition = position;
    climberMotor.setPosition(position);
  }

  public void stop() {
    climberMotor.stopMotor();
  }
}
