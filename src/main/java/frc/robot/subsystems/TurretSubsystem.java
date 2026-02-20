package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AprilTagSystem;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.UniverseConstants;
import frc.robot.subsystems.drive.Drive;

public class TurretSubsystem extends SubsystemBase {
  private final TalonFX turnMotor;
  private final TalonFX hoodMotor;
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final TalonFX feedMotor;
  private final TalonFX indexMotor;

  private boolean autoTurn = true;

  AprilTagSystem aprilTagSystem = new AprilTagSystem();
  Drive drive;
  private final DutyCycleOut duty = new DutyCycleOut(0);
  private final PositionDutyCycle positionRequest = new PositionDutyCycle(0);
  private final VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0);

  public TurretSubsystem(Drive drive) {
    this.drive = drive;
    turnMotor = new TalonFX(TurretConstants.TURN_MOTOR);
    hoodMotor = new TalonFX(TurretConstants.HOOD_MOTOR);
    leftMotor = new TalonFX(TurretConstants.LEFT_MOTOR);
    rightMotor = new TalonFX(TurretConstants.RIGHT_MOTOR);
    feedMotor = new TalonFX(TurretConstants.FEEDER_MOTOR);
    indexMotor = new TalonFX(TurretConstants.INDEX_MOTOR);

    configHood();
    configTurret();
    configFlywheels();
    configFeeders();
  }

  private void configHood() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 15;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 15;

    config.Feedback.RotorToSensorRatio = 75;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // TUNE PID
    config.Slot0.kP = 2;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;

    SoftwareLimitSwitchConfigs limits = new SoftwareLimitSwitchConfigs();
    limits.ForwardSoftLimitEnable = true;
    limits.ForwardSoftLimitThreshold = 6;
    limits.ReverseSoftLimitEnable = true;
    limits.ReverseSoftLimitThreshold = 0;

    config.SoftwareLimitSwitch = limits;

    hoodMotor.getConfigurator().apply(config);
  }

  private void configTurret() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 30;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // TUNE PID
    config.Slot0.kP = 2;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;

    turnMotor.getConfigurator().apply(config);
  }

  private void configFlywheels() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 60;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // TUNE PID
    config.Slot0.kP = 2;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;

    leftMotor.getConfigurator().apply(config);

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightMotor.getConfigurator().apply(config);
  }

  private void configFeeders() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 60;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    feedMotor.getConfigurator().apply(config);
    indexMotor.getConfigurator().apply(config);
  }

  public double getHoodPosition() {
    return hoodMotor.getPosition().getValueAsDouble();
  }

  public void turn(double speed) {
    turnMotor.setControl(duty.withOutput(speed));
  }

  public void turnToPosition(double turretPosition, double speed) {
    turnMotor.setControl(positionRequest.withPosition(turretPosition).withVelocity(speed));
  }

  public void turnToAngle(double hoodPosition, double speed) {
    hoodMotor.setControl(positionRequest.withPosition(hoodPosition).withVelocity(speed));
  }

  public void hood(double speed) {
    hoodMotor.setControl(duty.withOutput(-speed));
  }

  public void setFlywheelVelocity(double rps) {
    leftMotor.setControl(duty.withOutput(rps));
    rightMotor.setControl(duty.withOutput(rps));
  }

  public void setFeeder(double speed) {
    feedMotor.setControl(duty.withOutput(speed));
  }

  public void setIndexer(double speed) {
    indexMotor.setControl(duty.withOutput(speed));
  }

  public void changeAutoTurn(boolean turn) {
    autoTurn = turn;
  }

  public void stopAll() {
    turnMotor.stopMotor();
    hoodMotor.stopMotor();
    leftMotor.stopMotor();
    rightMotor.stopMotor();
    feedMotor.stopMotor();
    indexMotor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood Rotations", hoodMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Turret Rotations", turnMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Flywheel Velocity", leftMotor.getVelocity().getValueAsDouble());

    // Keeping turret pointed at hub, formula would be:
    // Take x and y difference btween coords of hub/robo, inverse tan for angle and subtract robot
    // heading to find
    // angle the turret needs to be at relative to robot

    // Use boolean to decide when to use it, when not to. Don't care about how much it can turn,
    // just leave that for later
    if (autoTurn) {
      Pose2d robotPose = drive.getPose();
      double angleReq =
          Math.toDegrees(
                  Math.atan2(
                      UniverseConstants.redhubY - robotPose.getY(),
                      UniverseConstants.redhubX - robotPose.getX()))
              - robotPose.getRotation().getDegrees();
      SmartDashboard.putNumber("Angle to turn to", angleReq);
      SmartDashboard.putNumber(
          "atan",
          Math.toDegrees(
              Math.atan2(
                  UniverseConstants.redhubY - robotPose.getY(),
                  UniverseConstants.redhubX - robotPose.getX())));
    }
  }
}
