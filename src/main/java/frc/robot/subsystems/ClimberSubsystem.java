package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  private final TalonFX climberMotor;
  private final DutyCycleOut dutyRequest = new DutyCycleOut(0);
  private final PositionDutyCycle holdRequest = new PositionDutyCycle(0);

  private double holdPosition = 0.0;
  private final double deadband = 0.02;

  public ClimberSubsystem() {

    climberMotor = new TalonFX(20);

    TalonFXConfiguration config = new TalonFXConfiguration();

    // Tune
    config.Slot0.kP = 1;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;

    config.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Brake;

    climberMotor.getConfigurator().apply(config);

    holdPosition = climberMotor.getPosition().getValueAsDouble();
  }

  public void runClimber(double joystickInput) {

    double output = MathUtil.applyDeadband(joystickInput, deadband);

    if (Math.abs(output) > 0) {
      climberMotor.setControl(dutyRequest.withOutput(output));
      holdPosition = climberMotor.getPosition().getValueAsDouble();
    } else {
      climberMotor.setControl(holdRequest.withPosition(holdPosition));
    }
  }

  public void stop() {
    holdPosition = climberMotor.getPosition().getValueAsDouble();
    climberMotor.setControl(holdRequest.withPosition(holdPosition));
  }
}
