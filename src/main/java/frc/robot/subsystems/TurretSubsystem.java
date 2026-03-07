package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
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
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AprilTagSystem;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.UniverseConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.NoSuchElementException;

public class TurretSubsystem extends SubsystemBase {
  StructPublisher<Pose2d> turretPosePublisher =
      NetworkTableInstance.getDefault().getStructTopic("TurretPose", Pose2d.struct).publish();

  private Rotation2d lastAngle = new Rotation2d();
  private Rotation2d continuousAngle = new Rotation2d();
  private final TalonFX azimuthMotor;
  private final TalonFX hoodMotor;
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final TalonFX feedMotor;
  private final TalonFX indexMotor;

  AprilTagSystem aprilTagSystem = new AprilTagSystem();
  Drive drive;
  boolean alliance = false;
  private final CANcoder encoder;
  private final DutyCycleOut duty = new DutyCycleOut(0);
  private final PositionDutyCycle positionRequest = new PositionDutyCycle(0);
  private final VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0);

  private InterpolatingDoubleTreeMap map1 = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap map2 = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap map3 = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap map4 = new InterpolatingDoubleTreeMap();

  private InterpolatingDoubleTreeMap tofmap1 = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap tofmap2 = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap tofmap3 = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap tofmap4 = new InterpolatingDoubleTreeMap();

  private double RPS1 = 33.0;
  private double RPS2 = 36.0;
  private double RPS3 = 41.0;
  private double RPS4 = 45.0;

  private boolean setWheels = false;

  private double goalPoseX;
  private double goalPoseY;

  public TurretSubsystem(Drive drive) {
    this.drive = drive;
    azimuthMotor = new TalonFX(TurretConstants.TURN_MOTOR);
    hoodMotor = new TalonFX(TurretConstants.HOOD_MOTOR);
    leftMotor = new TalonFX(TurretConstants.LEFT_MOTOR);
    rightMotor = new TalonFX(TurretConstants.RIGHT_MOTOR);
    feedMotor = new TalonFX(TurretConstants.FEEDER_MOTOR);
    indexMotor = new TalonFX(TurretConstants.INDEXER_MOTOR);
    encoder = new CANcoder(53);
    lastAngle = Rotation2d.fromDegrees(encoder.getAbsolutePosition().getValueAsDouble() * 360);
    configHood();
    configTurret();
    configFlywheels();
    configFeeders();

    // (distance in meters, time in seconds)
    tofmap1.put(2.00, 0.947);
    tofmap1.put(2.71, 0.807);

    tofmap2.put(2.97, 1.067);
    tofmap2.put(3.44, 0.900);

    tofmap3.put(3.54, 1.297);
    tofmap3.put(4.45, 1.101);

    tofmap4.put(4.96, 1.333);

    // (distance in meters, hood in rotations)
    map1.put(1.995, 0.05);
    map1.put(2.213, 0.625);
    map1.put(2.43, 0.95);
    map1.put(2.75, 2.626);

    map2.put(2.751, 1.04);
    map2.put(2.98, 1.25);
    map2.put(3.27, 3.33);

    map3.put(3.48, 0.43);
    map3.put(4.18, 1.82);
    map3.put(4.45, 2.748);

    map4.put(4.55, 1.249);
    map4.put(4.766, 1.453);
    map4.put(5.0, 1.686);
    map4.put(5.67, 3.20);
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      goalPoseX = UniverseConstants.redGoalPose.getX();
      goalPoseY = UniverseConstants.redGoalPose.getY();
      alliance = true;
    } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
      goalPoseX = UniverseConstants.blueGoalPose.getX();
      goalPoseY = UniverseConstants.blueGoalPose.getY();
      alliance = true;
    }
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
    // Configure CANcoder
    CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    cc_cfg.MagnetSensor.MagnetOffset =
        -0.1298828125; // Adjust this value based on your magnet alignment
    encoder.getConfigurator().apply(cc_cfg);

    // Configure TalonFX with fused CANcoder
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 30;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    config.Feedback.RotorToSensorRatio = 50; // Adjust based on your gear ratio

    SoftwareLimitSwitchConfigs limits = new SoftwareLimitSwitchConfigs();
    limits.ForwardSoftLimitEnable = true;
    limits.ForwardSoftLimitThreshold = 0.75;
    limits.ReverseSoftLimitEnable = true;
    limits.ReverseSoftLimitThreshold = -0.75;
    config.SoftwareLimitSwitch = limits;

    // TUNE PID
    config.Slot0.kP = 9;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;

    azimuthMotor.getConfigurator().apply(config);
  }

  private void configFlywheels() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 60;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // TUNE PID
    config.Slot0.kP = 0.045;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    config.Slot0.kV = 0.011; // Tested at Dist 1.765576281608437

    leftMotor.getConfigurator().apply(config);

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightMotor.getConfigurator().apply(config);
  }

  private void configFeeders() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    feedMotor.getConfigurator().apply(config);
    indexMotor.getConfigurator().apply(config);
  }

  public void feed(double speed) {
    setFeeder(speed);
    setIndexer(-speed);
  }

  public Command moveToPosition(Rotation2d targetAngle) {
    return Commands.run(() -> setTurretAzimuth(targetAngle), this)
        .until(() -> atTarget(targetAngle));
  }

  public boolean atTarget(Rotation2d targetAngle) {
    double currentPosition = azimuthMotor.getPosition().getValueAsDouble();
    double targetPosition = targetAngle.getRotations();
    double error = Math.abs(targetPosition - currentPosition);
    return error < 0.1;
  }

  public boolean hoodAtTarget(double position) {
    double current = hoodMotor.getPosition().getValueAsDouble();
    double error = Math.abs(position - current);
    return error < 0.1;
  }

  public void turn(double speed) {
    azimuthMotor.setControl(duty.withOutput(speed));
  }

  public void setTurretAzimuth(Rotation2d targetAngle) {
    double target = targetAngle.getRotations();
    double clampedTarget = optimizeTurretPosition(target);
    SmartDashboard.putNumber("Clamped Tartget", clampedTarget);
    azimuthMotor.setControl(positionRequest.withPosition(clampedTarget));
  }

  private double optimizeTurretPosition(double targetPosition) {
    final double FORWARD_LIMIT = TurretConstants.TURRET_FORWARD_LIMIT;
    final double REVERSE_LIMIT = TurretConstants.TURRET_REVERSE_LIMIT;

    // Get current position
    double currentPosition = azimuthMotor.getPosition().getValueAsDouble();

    // Clamp target to the valid range [-0.5, 0.5]
    double clampedTarget = MathUtil.clamp(targetPosition, -0.5, 0.5);

    // Try the target position as-is and with ±1 rotation offset
    double[] candidates = {clampedTarget, clampedTarget + 1.0, clampedTarget - 1.0};

    // Find the candidate that is within hardware limits and requires shortest distance
    double bestPosition = clampedTarget;
    double shortestDistance = Double.MAX_VALUE;

    for (double candidate : candidates) {
      // Check if within hardware limits
      if (candidate >= REVERSE_LIMIT && candidate <= FORWARD_LIMIT) {
        // Calculate distance from current position
        double distance = Math.abs(candidate - currentPosition);
        if (distance < shortestDistance) {
          shortestDistance = distance;
          bestPosition = candidate;
        }
      }
    }

    return bestPosition;
  }

  public void setHoodAngle(double targetPosition) {
    hoodMotor.setControl(positionRequest.withPosition(targetPosition));
  }

  public Command setTurretAngleCommand(double hoodPosition) {
    return Commands.run(() -> setHoodAngle(hoodPosition), this)
        .until(() -> hoodAtTarget(hoodPosition));
  }

  /**
   * @param robotPose
   * @param goalPose
   * @return an Roation2D from −180°, 180°
   */
  public Rotation2d calculateTurretAzimuth(Pose2d robotPose, Pose2d goalPose) {
    Transform2d robotToTurret =
        new Transform2d(new Translation2d(0.1397, 0.0), new Rotation2d()); // 5.5 inches
    Pose2d turretPose = robotPose.transformBy(robotToTurret);
    Translation2d turretToGoal = goalPose.getTranslation().minus(turretPose.getTranslation());
    Rotation2d fieldAngle = turretToGoal.getAngle();
    turretPosePublisher.set(turretPose);

    return fieldAngle
        .minus(robotPose.getRotation())
        .plus(new Rotation2d(Math.PI / 2)); // Adjust for turret 90 degree offset angle
  }

  public Pose2d predictFuturePose(Pose2d pose, double latencySeconds) {
    latencySeconds = latencySeconds * -1;
    ChassisSpeeds currentSpeed = drive.getFieldRelativeSpeeds();
    SmartDashboard.putNumber("Currnent Speed VX", currentSpeed.vxMetersPerSecond);
    SmartDashboard.putNumber("Currnent Speed VY", currentSpeed.vyMetersPerSecond);
    return pose.exp(
        new Twist2d(
            currentSpeed.vxMetersPerSecond * latencySeconds,
            currentSpeed.vyMetersPerSecond * latencySeconds,
            0));
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
    hoodMotor.setControl(duty.withOutput(-speed));
  }

  public void startHoldPivot() {
    double holdPivotRot = hoodMotor.getPosition().getValueAsDouble();
    hoodMotor.setControl(positionRequest.withPosition(holdPivotRot));
  }

  public void setFlywheelVelocity(double rps) {
    SmartDashboard.putNumber("RPS target", rps);
    leftMotor.setControl(velocityRequest.withVelocity(rps).withEnableFOC(true));
    rightMotor.setControl(velocityRequest.withVelocity(rps).withEnableFOC(true));
  }

  public void setFeeder(double speed) {
    feedMotor.setControl(duty.withOutput(speed).withEnableFOC(true));
  }

  public void setIndexer(double speed) {
    indexMotor.setControl(duty.withOutput(speed).withEnableFOC(true));
  }

  public void stopAll() {
    azimuthMotor.stopMotor();
    hoodMotor.stopMotor();
    leftMotor.stopMotor();
    rightMotor.stopMotor();
    feedMotor.stopMotor();
  }

  public void stopFlywheels() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public Command aautoAim() {
    // double xPosition = drive.getPose().getX();
    // double turretCurAngle =
    //     turretPositionToDegrees(encoder.getAbsolutePosition().getValueAsDouble());
    // if (xPosition >= 6 && DriverStation.getAlliance().get() == Alliance.Blue) {
    //   // feed towards field's 180 degrees
    //   // 180 - robot angle - turret angle
    //   return Commands.run(
    //       () -> {
    //         double angle = 180 - drive.getPose().getRotation().getDegrees() - turretCurAngle;
    //         turnToPosition(angle);
    //         setTurretAngle(5.5);
    //       },
    //       this);
    // }
    // if (xPosition <= 10.5 && DriverStation.getAlliance().get() == Alliance.Red) {
    //   // also feed, towards field's 0 degrees
    //   // 0 - robot angle - turret angle
    //   return Commands.run(
    //       () -> {
    //         double angle = 0 - drive.getPose().getRotation().getDegrees() - turretCurAngle;
    //         turnToPosition(angle);
    //         setTurretAngle(5.5);
    //       },
    //       this);
    // }
    return Commands.run(
        () -> {
          Pose2d robotPose = drive.getPose();
          Pose2d goal = new Pose2d(goalPoseX, goalPoseY, new Rotation2d());
          double dist = getDistance(robotPose, goal);
          InterpolatingDoubleTreeMap flightTimeMap;

          if (dist <= 2.75) {
            flightTimeMap = tofmap1;
          } else if (dist <= 3.5) {
            flightTimeMap = tofmap2;
          } else if (dist <= 4.5) {
            flightTimeMap = tofmap3;
          } else {
            flightTimeMap = tofmap4;
          }
          double scalingFactor = 1.0;
          double flightTime = flightTimeMap.get(dist);
          double latency = 0;
          double totalPredictionTime = (flightTime + latency) * scalingFactor;
          Pose2d predictedPose = predictFuturePose(robotPose, totalPredictionTime);
          double newDist = getDistance(predictedPose, goal);

          if (newDist <= 2.75) {
            flightTimeMap = tofmap1;
          } else if (dist <= 3.5) {
            flightTimeMap = tofmap2;
          } else if (dist <= 4.5) {
            flightTimeMap = tofmap3;
          } else {
            flightTimeMap = tofmap4;
          }

          double updatedFlightTime = flightTimeMap.get(newDist);
          Pose2d updatedPose = predictFuturePose(robotPose, updatedFlightTime);
          double updatedDist = getDistance(updatedPose, goal);

          double hoodPos = getHoodFromDistance(updatedDist);
          Rotation2d turretAngle = calculateTurretAzimuth(updatedPose, goal);

          setTurretAzimuth(turretAngle);
          setHoodAngle(hoodPos);
          SmartDashboard.putNumber("TURRET ROT", turretAngle.getRotations());
          SmartDashboard.putNumber("TURRET DEG", turretAngle.getDegrees());
        },
        this);
  }

  public Command autoAim() {
    return Commands.run(
        () -> {
          Pose2d robotPose = drive.getPose();

          Pose2d goal = new Pose2d(goalPoseX, goalPoseY, new Rotation2d());

          Pose2d futurePose = predictFuturePose(robotPose, 0.9);

          double dist = getDistance(futurePose, goal);

          Rotation2d turretAngle = calculateTurretAzimuth(futurePose, goal);

          double hoodPos = getHoodFromDistance(dist);

          setTurretAzimuth(turretAngle);
          setHoodAngle(hoodPos);
        },
        this);
  }

  public Command aimTest() {
    return Commands.run(
        () -> {
          Pose2d robotPose = drive.getPose();

          Pose2d goal = new Pose2d(goalPoseX, goalPoseY, new Rotation2d());

          Rotation2d turretAngle = calculateTurretAzimuth(robotPose, goal);

          setTurretAzimuth(turretAngle);
        },
        this);
  }

  public double hoodPositionToDegrees(double position) {
    return 5.0 * position + 20.0;
  }

  public double hoodDegreesToPosition(double degrees) {
    return (degrees - 20.0) / 5.0;
  }

  public double getDistance(Pose2d robotPose, Pose2d goalPose) {
    Transform2d robotToTurret =
        new Transform2d(new Translation2d(0.1397, 0.0), new Rotation2d()); // 5.5 inches in meters

    Pose2d turretPose = robotPose.plus(robotToTurret);

    return turretPose.getTranslation().getDistance(goalPose.getTranslation());
  }

  public double getHoodFromDistance(double distance) {
    InterpolatingDoubleTreeMap selectedMap;
    double flywheelRPS;

    // Determine speed zone
    if (distance <= 2.75) { // Zone 1
      selectedMap = map1;
      flywheelRPS = RPS1;
    } else if (distance <= 3.5) { // Zone 2
      selectedMap = map2;
      flywheelRPS = RPS2;
    } else if (distance <= 4.5) { // Zone 3
      selectedMap = map3;
      flywheelRPS = RPS3;
    } else { // Zone 4
      selectedMap = map4;
      flywheelRPS = RPS4;
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
    SmartDashboard.putNumber("Turret Rotations", azimuthMotor.getPosition().getValueAsDouble());
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

    Rotation2d turretAngle =
        calculateTurretAzimuth(
            drive.getPose(),
            new Pose2d(
                UniverseConstants.redGoalPose.getX(),
                UniverseConstants.redGoalPose.getY(),
                new Rotation2d()));

    SmartDashboard.putNumber("Degree turret go to", turretAngle.getRotations());

    double hoodPos = getHoodFromDistance(dist);
    SmartDashboard.putNumber("Hood position go to", hoodPos);

    // double hoodDeg =
    //     Math.toDegrees(calculateHoodAngle(drive.getPose(), UniverseConstants.redGoalPose));
    // SmartDashboard.putNumber("Degree hood go to", hoodDeg);

    Rotation2d currentAngle =
        Rotation2d.fromDegrees(encoder.getAbsolutePosition().getValueAsDouble() * 360);
    Rotation2d delta = currentAngle.minus(lastAngle);

    continuousAngle = continuousAngle.plus(delta);
    // Clamp continuous angle between -270 and 270 degrees
    double continuousDegrees = continuousAngle.getDegrees();
    continuousDegrees = MathUtil.clamp(continuousDegrees, -270, 270);
    continuousAngle = Rotation2d.fromDegrees(continuousDegrees);
    lastAngle = currentAngle;
    SmartDashboard.putNumber("Turret angle", azimuthMotor.getPosition().getValue().abs(Degree));
    if (!alliance) {
      try {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
          goalPoseX = UniverseConstants.redGoalPose.getX();
          goalPoseY = UniverseConstants.redGoalPose.getY();
          alliance = true;
        } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
          goalPoseX = UniverseConstants.blueGoalPose.getX();
          goalPoseY = UniverseConstants.blueGoalPose.getY();
          alliance = true;
        }
      } catch (NoSuchElementException nsee) {
        System.out.println("whaaaaaaaaa");
      }
    }
  }
}
