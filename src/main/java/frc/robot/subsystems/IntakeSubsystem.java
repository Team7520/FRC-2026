package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX intakeMotor;
  private final TalonFX extendMotor;
  private final DutyCycleOut duty = new DutyCycleOut(0);
  private final PositionDutyCycle pivotPosReq = new PositionDutyCycle(0);
  double extendedPosition = -16.4794921875;
  double retractedPosition = 0;

  public IntakeSubsystem() {
    intakeMotor = new TalonFX(57);
    extendMotor = new TalonFX(58); // placeholder ids

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 2;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 40.0;

    intakeMotor.getConfigurator().apply(config);
    intakeMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 10;
    extendMotor.getConfigurator().apply(config);
    extendMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
  }

  public void runIntake(double speed) {
    intakeMotor.setControl(duty.withOutput(speed));
  }

  public void extendSpin(double speed) {
    extendMotor.setControl(duty.withOutput(speed));
  }

  public void extend() {
    extendMotor.setControl(pivotPosReq.withPosition(extendedPosition));
  }

  public void retract() {
    extendMotor.setControl(pivotPosReq.withPosition(retractedPosition));
  }

  public void stopAll() {
    intakeMotor.setControl(duty.withOutput(0));
    extendMotor.setControl(duty.withOutput(0));
  }

  public Command extendIntake() {
    return Commands.run(() -> extend(), this).until(() -> atTarget(extendedPosition));
  }

  public Command retractIntake() {
    return Commands.run(() -> retract(), this).until(() -> atTarget(retractedPosition));
  }

  public boolean atTarget(double position) {
    System.out.print("EJoifSFHEOIFHEOGHEGJEJGEIGJ");

    double current = extendMotor.getPosition().getValueAsDouble();
    double error = Math.abs(position - current);
    System.out.print(error);
    return error < 0.1;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Position", extendMotor.getPosition().getValueAsDouble());
  }
}
