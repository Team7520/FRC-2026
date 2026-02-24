package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX climberMotor;
  private final DutyCycleOut duty = new DutyCycleOut(0);

  public ClimberSubsystem() {
    climberMotor = new TalonFX(100);
  }
}
