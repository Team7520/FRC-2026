package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AprilTagSystem;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.UniverseConstants;
import frc.robot.subsystems.drive.Drive;

public class TurretSubsystem extends SubsystemBase {
  private boolean pivotHolding = false;
  private double holdPivotRot = 0.0;
  private final TalonFX turnMotor;
  private final TalonFX hoodMotor;
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final TalonFX feedMotor;
  private final TalonFX indexMotor;

  private boolean autoTurn = true;

  AprilTagSystem aprilTagSystem = new AprilTagSystem();
  Drive drive;
  double rot;
  private final CANcoder encoder;
  private final DutyCycleOut duty = new DutyCycleOut(0);
  private final PositionDutyCycle positionRequest = new PositionDutyCycle(0);
  private final VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0);

  private InterpolatingDoubleTreeMap map1 = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap map2 = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap map3 = new InterpolatingDoubleTreeMap();

  private double RPS1 = 33.0;
  private double RPS2 = 36.0;
  private double RPS3 = 40.0;

  private boolean setWheels = false;

  public TurretSubsystem(Drive drive) {
    this.drive = drive;
    turnMotor = new TalonFX(TurretConstants.TURN_MOTOR);
    hoodMotor = new TalonFX(TurretConstants.HOOD_MOTOR);
    leftMotor = new TalonFX(TurretConstants.LEFT_MOTOR);
    rightMotor = new TalonFX(TurretConstants.RIGHT_MOTOR);
    feedMotor = new TalonFX(TurretConstants.FEEDER_MOTOR);
    indexMotor = new TalonFX(TurretConstants.INDEXER_MOTOR);
    encoder = new CANcoder(53);
    configHood();
    configTurret();
    configFlywheels();
    configFeeders();
    map1.put(1.995, 0.05);
    map1.put(2.213, 0.625);
    map1.put(2.43, 0.95);
    map1.put(2.75, 2.626);

    map2.put(2.751, 1.04);
    map2.put(2.98, 1.25);
    map2.put(3.27, 3.33);

    map3.put(3.48, 0.43);
    map3.put(4.18, 1.82);
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

  public void turretWheels(boolean on) {
    setWheels = on;
  }

  private void configTurret() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 30;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    config.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();

    SoftwareLimitSwitchConfigs limits = new SoftwareLimitSwitchConfigs();
    limits.ForwardSoftLimitEnable = true;
    limits.ForwardSoftLimitThreshold = 0.5;
    limits.ReverseSoftLimitEnable = true;
    limits.ReverseSoftLimitThreshold = -0.5;
    config.SoftwareLimitSwitch = limits;

    // TUNE PID
    config.Slot0.kP = 3;
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

  public void feed(double speed) {
    setFeeder(speed);
    setIndexer(-speed);
  }

  public Command moveToPosition(double position) {
    return Commands.run(() -> turnToPosition(position), this).until(() -> atTarget(position));
  }

  public boolean atTarget(double position) {
    double current = turnMotor.getPosition().getValueAsDouble();
    double error = Math.abs(position - current);
    return error < 0.1;
  }

  public boolean hoodAtTarget(double position) {
    double current = hoodMotor.getPosition().getValueAsDouble();
    double error = Math.abs(position - current);
    return error < 0.1;
  }

  public void turn(double speed) {
    turnMotor.setControl(duty.withOutput(speed));
  }

  public void turnToPosition(double turretPosition) {
    turnMotor.setControl(positionRequest.withPosition(turretPosition));
  }

  public void setTurretAngle(double hoodPosition) {
    hoodMotor.setControl(positionRequest.withPosition(hoodPosition));
  }

  public Command setTurretAngleCommand(double hoodPosition) {
    return Commands.run(() -> setTurretAngle(hoodPosition), this)
        .until(() -> hoodAtTarget(hoodPosition));
  }

  public Rotation2d calculateTurretAngle(Pose2d robotPose, Pose2d goalPose) {
    Transform2d robotToTurret =
        new Transform2d(new Translation2d(0.1397, 0.0), new Rotation2d()); // 5.5 inches
    Pose2d turretPose = robotPose.transformBy(robotToTurret);
    Translation2d turretToGoal = goalPose.getTranslation().minus(turretPose.getTranslation());
    Rotation2d fieldAngle = turretToGoal.getAngle();

    return fieldAngle.minus(robotPose.getRotation()).plus(new Rotation2d(Math.PI / 2));
  }

  // public double calculateHoodAngle(Pose2d robotPose, Pose3d goalPose) {
  //   // Horizontal distance
  //   double dx = goalPose.getX() - robotPose.getX();
  //   double dy = goalPose.getY() - robotPose.getY();
  //   double d = Math.sqrt(dx * dx + dy * dy);

  //   // Vertical difference
  //   double h = goalPose.getZ() - TurretConstants.launchHeight;

  //   double v = TurretConstants.closeLaunchSpeed;
  //   double v2 = v * v;

  //   // Discriminant of ballistic equation
  //   double discriminant =
  //       v2 * v2 - UniverseConstants.g * (UniverseConstants.g * d * d + 2 * h * v2);

  //   if (discriminant < 0) {
  //     return Double.NaN; // No physical solution
  //   }

  //   double sqrtDisc = Math.sqrt(discriminant);

  //   // Two possible launch angles
  //   double tanTheta1 = (v2 + sqrtDisc) / (UniverseConstants.g * d);
  //   double tanTheta2 = (v2 - sqrtDisc) / (UniverseConstants.g * d);

  //   double theta1 = Math.atan(tanTheta1);
  //   double theta2 = Math.atan(tanTheta2);

  //   double minAngle = Math.toRadians(45);

  //   // Choose the valid angle ≥ 45°
  //   double chosen = Double.NaN;
  //   if (theta1 >= minAngle) chosen = theta1;
  //   if (theta2 >= minAngle && (Double.isNaN(chosen) || theta2 > chosen)) chosen = theta2;

  //   return chosen; // radians
  // }

  public void hood(double speed) {
    pivotHolding = false;
    hoodMotor.setControl(duty.withOutput(-speed));
  }

  public void startHoldPivot() {
    holdPivotRot = hoodMotor.getPosition().getValueAsDouble();
    hoodMotor.setControl(positionRequest.withPosition(holdPivotRot));
    pivotHolding = true;
  }

  public void setFlywheelVelocity(double rps) {
    System.out.println(rps);
    leftMotor.setControl(velocityRequest.withVelocity(rps));
    rightMotor.setControl(velocityRequest.withVelocity(rps));
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
  }

  public void stopFlywheels() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public Command autoAim() {
    return Commands.run(
        () -> {
          Pose2d robotPose = drive.getPose();

          Pose2d goal =
              new Pose2d(
                  UniverseConstants.redGoalPose.getX(),
                  UniverseConstants.redGoalPose.getY(),
                  new Rotation2d());

          double distance = getDistance(robotPose, goal);

          ChassisSpeeds speeds = drive.getFieldRelativeSpeeds();

          Translation2d robotVelocity =
              new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
          double ballSpeed = 2; // tune this
          double time = distance / ballSpeed;
          Translation2d lead = robotVelocity.times(time);

          Translation2d adjustedGoal = goal.getTranslation().minus(lead);

          Pose2d virtualGoal = new Pose2d(adjustedGoal, new Rotation2d());
          double turretDeg = calculateTurretAngle(robotPose, virtualGoal).getDegrees();
          double turretPos = turretDegreesToPosition(turretDeg);

          double adjustedDistance = robotPose.getTranslation().getDistance(adjustedGoal);

          double hoodPos = getHoodForDistance(adjustedDistance);

          turnToPosition(turretPos);
          setTurretAngle(hoodPos);
        },
        this);
  }

  public double hoodPositionToDegrees(double position) {
    double deg = 5.0 * position + 20.0;
    return deg;
  }

  public double hoodDegreesToPosition(double degrees) {
    double position = (degrees - 20.0) / 5.0;
    return position;
  }

  public double turretPositionToDegrees(double position) {
    double deg = position / 0.5 * 180;
    return deg;
  }

  public double turretDegreesToPosition(double degrees) {
    double pos = degrees / 180 * 0.5;
    return pos;
  }

  public double getDistance(Pose2d robotPose, Pose2d goalPose) {
    Transform2d robotToTurret =
        new Transform2d(new Translation2d(0.1397, 0.0), new Rotation2d()); // 5.5 inches in meters

    Pose2d turretPose = robotPose.plus(robotToTurret);

    return turretPose.getTranslation().getDistance(goalPose.getTranslation());
  }

  public double getHoodForDistance(double distance) {
    InterpolatingDoubleTreeMap selectedMap;
    double flywheelRPS;

    // Determine speed zone
    if (distance <= 2.75) { // Zone 1
      selectedMap = map1;
      flywheelRPS = RPS1;
    } else if (distance <= 3.5) { // Zone 2
      selectedMap = map2;
      flywheelRPS = RPS2;
    } else { // Zone 3
      selectedMap = map3;
      flywheelRPS = RPS3;
    }
    if (setWheels) {
      setFlywheelVelocity(flywheelRPS);
    } else {
      stopFlywheels();
    }
    double hoodPos = selectedMap.get(distance);
    return hoodPos;
  }

  @Override
  public void periodic() {
    // if (pivotHolding) {
    //   hoodMotor.setControl(positionRequest.withPosition(holdPivotRot));
    // }
    SmartDashboard.putNumber(
        "Hood Angle Degrees", hoodPositionToDegrees(hoodMotor.getPosition().getValueAsDouble()));
    SmartDashboard.putNumber("Hood Rotations", hoodMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Turret Rotations", turnMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Flywheel Velocity", leftMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber(
        "Absolute Turret rotatations", encoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber(
        "Turret position rotatations", encoder.getPosition().getValueAsDouble());
    double dist =
        getDistance(
            drive.getPose(),
            new Pose2d(
                UniverseConstants.redGoalPose.getX(),
                UniverseConstants.redGoalPose.getY(),
                new Rotation2d()));
    SmartDashboard.putNumber("Distance to goal", dist);

    double turretDeg =
        calculateTurretAngle(
                drive.getPose(),
                new Pose2d(
                    UniverseConstants.redGoalPose.getX(),
                    UniverseConstants.redGoalPose.getY(),
                    new Rotation2d()))
            .getDegrees();

    rot = turretDegreesToPosition(turretDeg);
    SmartDashboard.putNumber("Degree turret go to", rot);

    double hoodPos = getHoodForDistance(dist);
    SmartDashboard.putNumber("Hood position go to", hoodPos);

    // double hoodDeg =
    //     Math.toDegrees(calculateHoodAngle(drive.getPose(), UniverseConstants.redGoalPose));
    // SmartDashboard.putNumber("Degree hood go to", hoodDeg);
  }
}
