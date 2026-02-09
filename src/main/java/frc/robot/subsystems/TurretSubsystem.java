package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
  private final TalonFX turnMotor;
  private final TalonFX hoodMotor;
  private final TalonFX topMotor;
  private final TalonFX bottomMotor;
  private final DutyCycleOut duty = new DutyCycleOut(0);

  public TurretSubsystem(int turnMotorId, int hoodMotorId, int topMotorId, int bottomMotorId) {
    turnMotor = new TalonFX(turnMotorId);
    hoodMotor = new TalonFX(hoodMotorId);
    topMotor = new TalonFX(topMotorId);
    bottomMotor = new TalonFX(bottomMotorId);

    // possibly add current limits
    turnMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
    hoodMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
    topMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
    bottomMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
  }

  public void turn(double speed) {
    turnMotor.setControl(duty.withOutput(speed));
  }

  public void hood(double speed) {
    hoodMotor.setControl(duty.withOutput(speed));
  }

  public void top(double speed) {
    topMotor.setControl(duty.withOutput(speed));
  }

  public void bottom(double speed) {
    bottomMotor.setControl(duty.withOutput(speed));
  }

  public void stopAll() {
    turnMotor.setControl(duty.withOutput(0));
    hoodMotor.setControl(duty.withOutput(0));
    topMotor.setControl(duty.withOutput(0));
    bottomMotor.setControl(duty.withOutput(0));
  }
}
