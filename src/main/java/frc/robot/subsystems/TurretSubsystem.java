package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
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

  public enum RobotZone {
    RED_SHOOTING,
    RED_FEEDING,
    RED_FAR_ZONE,
    BLUE_SHOOTING,
    BLUE_FEEDING,
    BLUE_FAR_ZONE,
    NO_ALLIANCE
  }

  StructPublisher<Pose2d> turretPosePublisher =
      NetworkTableInstance.getDefault().getStructTopic("TurretPose", Pose2d.struct).publish();

  private Rotation2d lastAngle = new Rotation2d();
  private Rotation2d continuousAngle = new Rotation2d();
  private final TalonFX azimuthMotor;
  private final TalonFX hoodMotor;
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final TalonFX feedMotor;
  private final TalonFX secondFeed;
  private final TalonFX indexMotor;
  private final StrictFollower follower;

  AprilTagSystem aprilTagSystem = new AprilTagSystem();
  Drive drive;
  Alliance currentAlliance = null;
  boolean availableAlliance = false;
  private final CANcoder encoder;
  private final DutyCycleOut duty = new DutyCycleOut(0);
  private final PositionDutyCycle positionRequest = new PositionDutyCycle(0);
  private final VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0);
  private final VelocityVoltage velocityVoltRequest = new VelocityVoltage(0);

  private InterpolatingDoubleTreeMap map1 = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap map2 = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap map3 = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap map4 = new InterpolatingDoubleTreeMap();

  private InterpolatingDoubleTreeMap tofmap1 = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap tofmap2 = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap tofmap3 = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap tofmap4 = new InterpolatingDoubleTreeMap();

  private double RPS1 = 32.0;
  private double RPS2 = 34.0;
  private double RPS3 = 38.0;
  private double RPS4 = 42.0;

  private boolean setWheels = false;
  private boolean feederToggle = false;

  private double goalPoseX;
  private double goalPoseY;

  public TurretSubsystem(Drive drive) {
    this.drive = drive;
    azimuthMotor = new TalonFX(TurretConstants.TURN_MOTOR);
    hoodMotor = new TalonFX(TurretConstants.HOOD_MOTOR);
    leftMotor = new TalonFX(TurretConstants.LEFT_MOTOR);
    rightMotor = new TalonFX(TurretConstants.RIGHT_MOTOR);
    feedMotor = new TalonFX(TurretConstants.FEEDER_MOTOR);
    follower = new StrictFollower(feedMotor.getDeviceID());
    indexMotor = new TalonFX(TurretConstants.INDEXER_MOTOR);
    secondFeed = new TalonFX(TurretConstants.SECOND_FEED);
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
    // if (DriverStation.getAlliance().get() == Alliance.Red) {
    //   goalPoseX = UniverseConstants.redGoalPose.getX();
    //   goalPoseY = UniverseConstants.redGoalPose.getY();
    //   alliance = true;
    // } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
    //   goalPoseX = UniverseConstants.blueGoalPose.getX();
    //   goalPoseY = UniverseConstants.blueGoalPose.getY();
    //   alliance = true;
    // }
  }

  private void configHood() {
    TalonFXConfiguration config = new TalonFXConfiguration();

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

  public void turretWheels(boolean set) {
    setWheels = set;
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
    config.Slot0.kP = 0.045 * 12;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    config.Slot0.kV = 0.0115 * 12; // Tested at Dist 1.765576281608437

    leftMotor.getConfigurator().apply(config);

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightMotor.getConfigurator().apply(config);
  }

  private void configFeeders() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    indexMotor.getConfigurator().apply(config);

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    feedMotor.getConfigurator().apply(config);

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    secondFeed.getConfigurator().apply(config);
    secondFeed.setControl(follower);
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

  public Pose2d predictFuturePose(Pose2d robotPose, double timeOfFlight, double odometryLatency) {
    ChassisSpeeds currentSpeed = drive.getFieldRelativeSpeeds();
    SmartDashboard.putNumber("Current Speed VX", currentSpeed.vxMetersPerSecond);
    SmartDashboard.putNumber("Current Speed VY", currentSpeed.vyMetersPerSecond);
    return new Pose2d(
        robotPose.getX() + currentSpeed.vxMetersPerSecond * (odometryLatency + timeOfFlight),
        robotPose.getY() + currentSpeed.vyMetersPerSecond * (timeOfFlight + odometryLatency),
        robotPose
            .getRotation()
            .plus(new Rotation2d(currentSpeed.omegaRadiansPerSecond * odometryLatency)));
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
    leftMotor.setControl(velocityVoltRequest.withVelocity(rps).withEnableFOC(true));
    rightMotor.setControl(velocityVoltRequest.withVelocity(rps).withEnableFOC(true));
  }

  public void setFeeder(double speed) {
    feedMotor.setControl(duty.withOutput(speed).withEnableFOC(true));
  }

  public void toggleFeeder() {
    if (feederToggle) {
      feederToggle = false;
      setFeeder(0);
    } else {
      feederToggle = true;
      setFeeder(1);
    }
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

  public void setToZero() {
    leftMotor.set(0);
    rightMotor.set(0);
  }

  public RobotZone getRobotZone() {
    if (!availableAlliance) {
      return RobotZone.NO_ALLIANCE;
    }

    double xPosition = drive.getPose().getX();
    Alliance alliance = DriverStation.getAlliance().get();

    if (alliance == Alliance.Red) {
      if (xPosition <= 6) {
        return RobotZone.RED_FAR_ZONE;
      } else if (xPosition <= 10.5) {
        return RobotZone.RED_FEEDING;
      } else {
        return RobotZone.RED_SHOOTING;
      }
    } else {
      if (xPosition >= 10.5) {
        return RobotZone.BLUE_FAR_ZONE;
      } else if (xPosition >= 6) {
        return RobotZone.BLUE_FEEDING;
      } else {
        return RobotZone.BLUE_SHOOTING;
      }
    }
  }

  public Command aautoAim() {
    return Commands.run(
        () -> {
          RobotZone zone = getRobotZone();
          SmartDashboard.putString("Robot Zone", zone.toString());

          switch (zone) {
            case RED_FAR_ZONE:
              {
                Rotation2d turretAngle =
                    new Rotation2d(
                        0 - drive.getPose().getRotation().getRadians() - Math.toRadians(-90));
                setTurretAzimuth(turretAngle);
                setHoodAngle(0);
                break;
              }
            case RED_FEEDING:
              {
                Rotation2d turretAngle =
                    new Rotation2d(
                        0 - drive.getPose().getRotation().getRadians() - Math.toRadians(-90));
                setTurretAzimuth(turretAngle);
                setHoodAngle(3);
                break;
              }
            case RED_SHOOTING:
              {
                Pose2d robotPose = drive.getPose();
                Pose2d goal = new Pose2d(goalPoseX, goalPoseY, new Rotation2d());
                double dist = getDistance(robotPose, goal);

                Pose2d currentPose = robotPose;
                double currentDist = dist;
                double odometryLatency = 0.125;

                double flightTime = 0.0226 * currentDist + 0.947;
                currentPose = predictFuturePose(robotPose, flightTime, odometryLatency);
                currentDist = getDistance(currentPose, goal);

                double hoodPos = getHoodFromDistance(currentDist);
                Rotation2d turretAngle = calculateTurretAzimuth(currentPose, goal);

                setTurretAzimuth(turretAngle);
                setHoodAngle(hoodPos);
                SmartDashboard.putNumber("TURRET ROT", turretAngle.getRotations());
                SmartDashboard.putNumber("TURRET DEG", turretAngle.getDegrees());
                break;
              }
            case BLUE_FAR_ZONE:
              {
                Rotation2d turretAngle =
                    new Rotation2d(
                        Math.PI - drive.getPose().getRotation().getRadians() - Math.toRadians(-90));
                setTurretAzimuth(turretAngle);
                setHoodAngle(0);
                break;
              }
            case BLUE_FEEDING:
              {
                Rotation2d turretAngle =
                    new Rotation2d(
                        Math.PI - drive.getPose().getRotation().getRadians() - Math.toRadians(-90));
                setTurretAzimuth(turretAngle);
                setHoodAngle(3);

                break;
              }
            case BLUE_SHOOTING:
              {
                Pose2d robotPose = drive.getPose();
                Pose2d goal = new Pose2d(goalPoseX, goalPoseY, new Rotation2d());
                double dist = getDistance(robotPose, goal);

                Pose2d currentPose = robotPose;
                double currentDist = dist;
                double odometryLatency = 0.125;

                double flightTime = 0.0226 * currentDist + 0.947;
                currentPose = predictFuturePose(robotPose, flightTime, odometryLatency);
                currentDist = getDistance(currentPose, goal);

                double hoodPos = getHoodFromDistance(currentDist);
                Rotation2d turretAngle = calculateTurretAzimuth(currentPose, goal);

                setTurretAzimuth(turretAngle);
                setHoodAngle(hoodPos);
                SmartDashboard.putNumber("TURRET ROT", turretAngle.getRotations());
                SmartDashboard.putNumber("TURRET DEG", turretAngle.getDegrees());
                break;
              }
            case NO_ALLIANCE:
              {
                System.out.println("Did nothing, alliance not selected yet!");
                break;
              }
          }
        },
        this);
  }

  public Command autoAim() {
    return Commands.run(
        () -> {
          Pose2d robotPose = drive.getPose();

          Pose2d goal = new Pose2d(goalPoseX, goalPoseY, new Rotation2d());

          Pose2d futurePose = predictFuturePose(robotPose, 0.9, 0.15);

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
    double scaleFactor = 0.54;
    // double flywheelRPS;

    // // Determine speed zone
    // if (distance <= 2.75) { // Zone 1
    //   selectedMap = map1;
    //   flywheelRPS = RPS1;
    // } else if (distance <= 3.5) { // Zone 2
    //   selectedMap = map2;
    //   flywheelRPS = RPS2;
    // } else if (distance <= 4.5) { // Zone 3
    //   selectedMap = map3;
    //   flywheelRPS = RPS3;
    // } else { // Zone 4
    //   selectedMap = map4;
    //   flywheelRPS = RPS4;
    // }

    if (setWheels) {
      RobotZone zone = getRobotZone();
      switch (zone) {
        case RED_FEEDING:
        case BLUE_FEEDING:
          distance += 2;
      }
      setFlywheelVelocity(getSpeedFromDistance(distance));
    } else {
      stopFlywheels();
    }
    double hoodPos = distance * scaleFactor;
    return hoodPos;
  }

  public double getSpeedFromDistance(double distance) {
    double b = 23.5;
    double rpsPerDistance = 3.6;
    double speed = rpsPerDistance * distance + b;
    return speed;
  }

  @Override
  public void periodic() {
    // if (pivotHolding) {
    //   hoodMotor.setControl(positionRequest.withPosition(holdPivotRot));
    // }
    SmartDashboard.putNumber(
        "Hood Angle Degrees", hoodPositionToDegrees(hoodMotor.getPosition().getValueAsDouble()));
    // SmartDashboard.putNumber("Turret Rotations", azimuthMotor.getPosition().getValueAsDouble());
    // SmartDashboard.putNumber(
    //     "Absolute Turret rotatations", encoder.getAbsolutePosition().getValueAsDouble());

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

    // SmartDashboard.putNumber("Degree turret go to", turretAngle.getRotations());

    double hoodPos = getHoodFromDistance(dist);
    // SmartDashboard.putNumber("Hood position go to", hoodPos);

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
    if (!availableAlliance) {
      try {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
          goalPoseX = UniverseConstants.redGoalPose.getX();
          goalPoseY = UniverseConstants.redGoalPose.getY();
          availableAlliance = true;
        } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
          goalPoseX = UniverseConstants.blueGoalPose.getX();
          goalPoseY = UniverseConstants.blueGoalPose.getY();
          availableAlliance = true;
        }
      } catch (NoSuchElementException nsee) {
        System.out.println("No available alliance!");
      }
    }
  }
}
