// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Add your docs here. */
public class AprilTagSystem extends SubsystemBase {

  public static class CameraInfo {
    public final String name;
    public final PhotonCamera camera;
    public boolean isOpen;
    public final Transform3d robotToCamera;

    public CameraInfo(String name, PhotonCamera camera, boolean isOpen, Transform3d robotToCamera) {
      this.name = name;
      this.camera = camera;
      this.isOpen = isOpen;
      this.robotToCamera = robotToCamera;
    }
  }

  public static class LimeInfo {
    public final String name;
    public boolean isOpen;
    public Transform3d position;

    public LimeInfo(String name, boolean isOpen, Transform3d position) {
      this.name = name;
      this.isOpen = isOpen;
      this.position = position;

      LimelightHelpers.setCameraPose_RobotSpace(
          name,
          position.getX(), // Forward offset (meters)
          position.getY(), // Side offset (meters)
          position.getZ(), // Height offset (meters)
          Units.radiansToDegrees(position.getRotation().getX()), // Roll (degrees)
          Units.radiansToDegrees(position.getRotation().getY()), // Pitch (degrees)
          Units.radiansToDegrees(position.getRotation().getZ()) // Yaw (degrees)
          );
    }
  }

  private final List<CameraInfo> cameraList = new ArrayList<>();
  private final List<LimeInfo> limes = new ArrayList<>();

  private final PipeLineType TYPE = PipeLineType.APRIL_TAG;
  private boolean allOpen = false;
  private boolean facingTarget = false;
  private AprilTagFieldLayout aprilTagFieldLayout;
  private List<AprilTag> apriltags;
  public boolean aprilTagLayoutLoaded = false;
  private final double MAX_RANGE = 20; // In meters, anything beyond 2 meters should not be used

  private Pose2d robotPose;
  private AprilTag closestTag;
  private List<PhotonPoseEstimator> estimators = null;

  private final String frontRight = "limelight-frontr";
  private final String frontLeft = "limelight-frontl";
  private final String backRight = "limelight-backr";

  public enum PipeLineType {
    APRIL_TAG,
    COLORED_SHAPE,
    REFLECTIVE
  }

  public AprilTagSystem() {
    // Initialize the cameras
    cameraList.add(
        new CameraInfo(
            "Cam1",
            new PhotonCamera("FrontRightCam"),
            false,
            new Transform3d(
                0.283940504, // X forward
                -0.188200919, // Y right
                0.205,
                new Rotation3d(
                    0.0,
                    Math.toRadians(15), // pitched up
                    0.0 // facing forward
                    ))));

    limes.add(
        new LimeInfo(
            frontLeft,
            false,
            new Transform3d(
                0.300942,
                -0.275542,
                0.076781,
                new Rotation3d(
                    Units.degreesToRadians(180),
                    Units.degreesToRadians(60),
                    Units.degreesToRadians(45)))));

    limes.add(
        new LimeInfo(
            frontRight,
            false,
            new Transform3d(
                0.300942,
                0.275542,
                0.076781,
                new Rotation3d(
                    Units.degreesToRadians(180),
                    Units.degreesToRadians(60),
                    Units.degreesToRadians(-45)))));

    limes.add(
        new LimeInfo(
            backRight,
            false,
            new Transform3d(
                0.279069,
                0.216958,
                0.167095,
                new Rotation3d(
                    Units.degreesToRadians(14.028),
                    Units.degreesToRadians(65),
                    Units.degreesToRadians(-121.321)))));
  }

  @Override
  public void periodic() {
    allOpen = true;
    for (int i = 0; i < cameraList.size(); i++) {
      if (cameraList.get(i).camera.isConnected()) {
        cameraList.get(i).isOpen = true;
      } else {
        allOpen = false;
      }
      SmartDashboard.putBoolean(cameraList.get(i).name + " OPEN?", cameraList.get(i).isOpen);
    }
    for (int i = 0; i < limes.size(); i++) {
      if (LimelightHelpers.getHeartbeat(limes.get(i).name) != 0) {
        limes.get(i).isOpen = true;
      } else {
        allOpen = false;
      }
      SmartDashboard.putBoolean(limes.get(i).name + " OPEN?", limes.get(i).isOpen);
    }

    SmartDashboard.putNumber("Closest cam", whichClosest());
    SmartDashboard.putNumber(limes.get(0).name + " Distance", getClosest(1));
    SmartDashboard.putNumber(limes.get(0).name + " Distance", getClosest(2));
    SmartDashboard.putNumber(limes.get(0).name + " Distance", getClosest(3));
    SmartDashboard.putNumber("Pi 1 Distance", getClosest(0));
  }

  public List<PhotonPoseEstimator> getEstimators() {
    // Ensure layout is loaded first
    if (!aprilTagLayoutLoaded) {
      initiateAprilTagLayout();
    }
    // Only build estimators once
    if (estimators == null && aprilTagLayoutLoaded) {
      estimators =
          cameraList.stream()
              .map(
                  info ->
                      new PhotonPoseEstimator(
                          aprilTagFieldLayout,
                          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                          info.robotToCamera))
              .collect(Collectors.toList());
    }
    return estimators == null ? List.of() : estimators;
  }

  public int whichClosest() {
    double closest = 100;
    int highest = -1;
    for (int i = 0; i < 5; i++) {
      double distance = getClosest(i);
      if (distance < closest && distance != -1) {
        closest = distance;
        highest = i;
      }
    }
    if (highest != -1) {
      return highest;
    }
    return -1;
  }
  /**
   * Loads field layout of april tags. The aprilTagFieldLayout field is initiated here because the
   * layout takes a while to load - often crashes if called too soon (such as within class
   * constructor).
   *
   * @return a boolean indicating whether the layout was loaded
   */
  public boolean initiateAprilTagLayout() {
    for (int i = 0; i < 5 && !aprilTagLayoutLoaded; i++) {
      try {
        aprilTagFieldLayout =
            AprilTagFieldLayout.loadFromResource(
                AprilTagFields.k2025ReefscapeAndyMark.m_resourceFile);
        apriltags = aprilTagFieldLayout.getTags();
        aprilTagLayoutLoaded = true;
      } catch (IOException e) {
        e.printStackTrace();
        System.out.println("Load April Tag Layout Error");
      }
    }
    return aprilTagLayoutLoaded;
  }

  public AprilTagFieldLayout getAprilTagLayout() {
    return aprilTagFieldLayout;
  }

  /**
   * Returns the distance of the tag to the given camera, used to determine which has the most
   * accurate data. Index 0 represent photonCamera, 1-3 represent Limelights
   *
   * @return a double representing the ambiguity of the camera
   */
  public double getClosest(int cameraIndex) {
    if (cameraIndex < 0 || cameraIndex > 3) {
      return -1; // Handle invalid camera index
    }

    PhotonPipelineResult result = null;
    double x, y, z, distance = 0;
    double[] offsets;

    if (cameraIndex == 0) {
      CameraInfo cameraInfo = cameraList.get(cameraIndex);
      result = getLatestCameraResult(cameraInfo.camera);

      if (result == null || !result.hasTargets()) {
        return -1; // Handle the case where no targets are found
      }

      PhotonTrackedTarget target = result.getBestTarget();
      Transform3d targets = target.getBestCameraToTarget();
      x = targets.getX();
      y = targets.getY();
      z = targets.getZ();
      distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
      return distance;

    } else {
      LimeInfo lime = limes.get(cameraIndex - 1);
      if (!LimelightHelpers.getTV(lime.name)) {
        return -1;
      }

      offsets = LimelightHelpers.getTargetPose_CameraSpace(lime.name);
      x = offsets[0];
      y = offsets[1];
      z = offsets[2];
      distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
      return distance;
    }
  }

  /**
   * Returns the capture time, used for pose estimator
   *
   * @return capture time in milliseconds
   */
  public double getCaptureTime() {
    CameraInfo cameraInfo;
    LimeInfo lime;
    PhotonPipelineResult result;
    int cam = whichClosest();
    if (cam == 0) {
      cameraInfo = cameraList.get(cam);
      result = cameraInfo.camera.getLatestResult();
      return result.getTimestampSeconds();
    } else if (cam != -1) {
      lime = limes.get(cam - 1);
      return Timer.getFPGATimestamp()
          - (LimelightHelpers.getLatency_Capture(lime.name) / 1000.0)
          - (LimelightHelpers.getLatency_Pipeline(lime.name) / 1000.0);
    }
    return -1;
  }

  /**
   * Estimates the current robot position based on the april tag it sees. April tags farther than
   * {@link #MAX_RANGE} are not considered.
   *
   * @return a Pose2d
   */
  public Pose2d getCurrentRobotFieldPose() {
    PhotonPipelineResult result = null;
    int cam2Use = whichClosest();
    if (cam2Use == 0) {
      result = getLatestCameraResult(cameraList.get(cam2Use).camera);
      Transform3d robotToCamera = cameraList.get(cam2Use).robotToCamera;
      PhotonTrackedTarget target = result.getBestTarget();
      if (target.getBestCameraToTarget().getX() > MAX_RANGE) {
        return null;
      }
      Pose3d robotPose =
          PhotonUtils.estimateFieldToRobotAprilTag(
              target.getBestCameraToTarget(),
              aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(),
              robotToCamera.inverse());
      return robotPose.toPose2d();
    } else if (cam2Use != -1) {
      return LimelightHelpers.getBotPose2d_wpiBlue(limes.get(cam2Use - 1).name);
    } else {
      return null;
    }
  }

  /**
   * Finds the closest visible AprilTag from all available cameras. Returns the field-relative pose
   * of that tag.
   *
   * @return Pose2d of the closest visible tag, or null if none seen
   */
  public Pose2d getClosestTagPose() {
    if (!aprilTagLayoutLoaded) {
      initiateAprilTagLayout();
    }
    if (!aprilTagLayoutLoaded) {
      System.out.println("!aprilTagLayoutLoaded");
      return null;
    }

    Pose2d closestTagPose = null;
    double closestDistance = Double.MAX_VALUE;

    for (CameraInfo cam : cameraList) {
      PhotonPipelineResult result = cam.camera.getLatestResult();
      if (result == null || !result.hasTargets()) {
        continue;
      }

      PhotonTrackedTarget target = result.getBestTarget();
      if (!aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
        continue;
      }

      double distance = target.getBestCameraToTarget().getTranslation().getNorm();
      if (distance > MAX_RANGE) {
        continue;
      }

      Pose3d tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get();
      if (distance < closestDistance) {
        closestDistance = distance;
        closestTagPose = tagPose.toPose2d();
        this.closestTag = new AprilTag(target.getFiducialId(), tagPose);
      }
    }

    return closestTagPose;
  }

  /**
   * Returns a new Pose2d offset from the given tag pose.
   *
   * @param tagPose The Pose2d of the AprilTag
   * @param forwardMeters Distance in front of the tag (+ forward, - behind)
   * @param lateralMeters Distance to the right of the tag (+ right, - left)
   * @return Offset Pose2d
   */
  public Pose2d getOffsetPose(Pose2d tagPose, double forwardMeters, double lateralMeters) {
    // Construct a transform in the tag's frame: forward + right
    Transform2d offset =
        new Transform2d(
            new Translation2d(forwardMeters, lateralMeters),
            Rotation2d.fromDegrees(180) // Keep original tag orientation
            );

    // Apply the transform relative to the tag pose
    return tagPose.plus(offset);
  }

  private static final Set<Integer> reefscapeTagIDs =
      Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);

  public Pose2d getNearestTagPose(Pose2d robotPose) {
    double minDistance = Double.MAX_VALUE;
    Pose2d closestPose = null;

    if (!aprilTagLayoutLoaded) {
      initiateAprilTagLayout();
    }
    for (AprilTag tag : aprilTagFieldLayout.getTags()) {
      if (!reefscapeTagIDs.contains(tag.ID)) continue; // filter only reef tags

      Pose2d tagPose =
          new Pose2d(tag.pose.getX(), tag.pose.getY(), tag.pose.getRotation().toRotation2d());

      double distance = robotPose.getTranslation().getDistance(tagPose.getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        closestPose = tagPose;
      }
    }

    return closestPose;
  }

  public Pose2d getAlignPose(Pose2d robotPose, Pose2d tagPose, boolean faceFront) {
    Translation2d toTag = tagPose.getTranslation().minus(robotPose.getTranslation());
    double angleToTag = Math.atan2(toTag.getY(), toTag.getX());
    Rotation2d desiredRotation = Rotation2d.fromRadians(angleToTag);

    if (!faceFront) {
      desiredRotation = desiredRotation.rotateBy(Rotation2d.fromDegrees(180));
    }

    return new Pose2d(tagPose.getTranslation(), desiredRotation);
  }

  public Pose2d getOptimalAlignPose(Pose2d robotPose, Pose2d poseA, Pose2d poseB) {

    double costA =
        robotPose.getTranslation().getDistance(poseA.getTranslation())
            + Math.abs(robotPose.getRotation().minus(poseA.getRotation()).getRadians());
    double costB =
        robotPose.getTranslation().getDistance(poseB.getTranslation())
            + Math.abs(robotPose.getRotation().minus(poseB.getRotation()).getRadians());
    return (costA <= costB) ? poseA : poseB;
  }

  private PhotonPipelineResult getLatestCameraResult(PhotonCamera camera) {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      return new PhotonPipelineResult();
    }
    return results.get(results.size() - 1);
  }
}
