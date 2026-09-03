package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX intakeMotor;
  private final TalonFX intakeFollowerMotor;
  private final TalonFX extendMotor;
  private final DutyCycleOut duty = new DutyCycleOut(0);
  private final PositionDutyCycle pivotPosReq = new PositionDutyCycle(0);
  double extendedPosition = -16;
  double retractedPosition = -5;
  private final double CURRENT_THRESHOLD = -27;

  public IntakeSubsystem() {
    intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR);
    intakeFollowerMotor = new TalonFX(IntakeConstants.FOLLOWER_MOTOR);
    extendMotor = new TalonFX(IntakeConstants.EXTEND_MOTOR);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 2;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 70;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 45;

    intakeMotor.getConfigurator().apply(config);
    intakeMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);

    // Both intake rollers use the same 70 A stator and 45 A supply limits.
    intakeFollowerMotor.getConfigurator().apply(config);
    intakeFollowerMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
    // Follower output updates depend on the leader's duty-cycle status signal.
    intakeMotor.getDutyCycle().setUpdateFrequency(50.0);
    intakeFollowerMotor.setControl(
        new Follower(intakeMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    config.Slot0.kP = 0.5;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 20;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 40;
    extendMotor.getConfigurator().apply(config);
    extendMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
  }

  public void runIntake(double speed) {
    intakeMotor.setControl(duty.withOutput(speed).withEnableFOC(true));
  }

  public void extendSpin(double speed) {
    extendMotor.setControl(duty.withOutput(speed));
  }

  public void extend() {
    // TalonFXConfiguration config = new TalonFXConfiguration();
    // config.CurrentLimits.StatorCurrentLimitEnable = true;
    // config.CurrentLimits.StatorCurrentLimit = 30;
    // extendMotor.getConfigurator().apply(config);
    extendMotor.setControl(pivotPosReq.withPosition(extendedPosition));
  }

  public void retractWithSpeed() {
    extendMotor.setControl(pivotPosReq.withPosition(retractedPosition).withVelocity(0.05));
  }

  public void retract() {
    // TalonFXConfiguration config = new TalonFXConfiguration();
    // config.CurrentLimits.StatorCurrentLimitEnable = true;
    // config.CurrentLimits.StatorCurrentLimit = 30;
    // extendMotor.getConfigurator().apply(config);
    extendMotor.setControl(pivotPosReq.withPosition(retractedPosition));
  }

  public void stopAll() {
    // Keep the second roller in follower mode; zeroing the leader stops both rollers.
    intakeMotor.setControl(duty.withOutput(0));
    extendMotor.setControl(duty.withOutput(0));
  }

  public void resetPosition(double position) {
    extendMotor.setPosition(position);
  }

  public double getExtendedPosition() {
    return extendedPosition;
  }

  public void setCoast() {
    // TalonFXConfiguration config = new TalonFXConfiguration();
    // config.CurrentLimits.StatorCurrentLimitEnable = true;
    // config.CurrentLimits.StatorCurrentLimit = 15;
    // extendMotor.getConfigurator().apply(config);
    extendMotor.setControl(new CoastOut());
  }

  public void setNeutralforCurrent() {
    //System.out.println("running neutral current");
    double currentDraw = extendMotor.getTorqueCurrent().getValueAsDouble();
    if (currentDraw <= CURRENT_THRESHOLD) {
      setCoast();
    }
  }

  public Command backDrive() {
    return Commands.run(() -> setNeutralforCurrent(), this);
  }

  public Command extendIntake() {
    return Commands.run(() -> extend()).until(() -> atTarget(extendedPosition));
    // .finallyDo(() -> setNeutral());
  }

  // public Command retractIntake() {
  //   return Commands.run(() -> retract(), this).until(() -> atTarget(retractedPosition));
  // }
  // public Command extendIntake() {
  //   return Commands.runOnce(() -> extend(), this);
  // }

  public Command retractIntake() {
    return Commands.runOnce(() -> retract());
  }

  public Command slowRetract() {
    return Commands.runOnce(() -> retractWithSpeed(), this);
  }

  public boolean atTarget(double position) {
    double current = extendMotor.getPosition().getValueAsDouble();
    double error = Math.abs(position - current);
    // System.out.print(error);
    return error < 0.1;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Position", extendMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber(
        "Intake deploy current", extendMotor.getTorqueCurrent().getValueAsDouble());
    if (this.getCurrentCommand() != null) {
      SmartDashboard.putString("current intake commaned", this.getCurrentCommand().getName());
    } else {
      SmartDashboard.putString("current intake commaned", "null");
    }
  }
}
