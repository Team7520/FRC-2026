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

  private final List<CameraInfo> cameraList = new ArrayList<>();

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

  private final String limelight1 = "limelight-one";
  private final String limelight2 = "limelight-two";
  private final String limelight3 = "limelight-three";

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

    cameraList.add(
        new CameraInfo(
            "Cam2",
            new PhotonCamera("Cam2"),
            false,
            new Transform3d(
                0.283940504, // X forward
                0.188200919, // Y left
                0.205,
                new Rotation3d(0.0, Math.toRadians(15), 0.0))));
  }

  @Override
  public void periodic() {
    allOpen = true;
    boolean lime1 = true;
    boolean lime2 = true;
    boolean lime3 = true;
    for (int i = 0; i < cameraList.size(); i++) {
      if (cameraList.get(i).camera.isConnected()) {
        cameraList.get(i).isOpen = true;
      } else {
        cameraList.get(i).isOpen = false;
        allOpen = false;
        System.out.printf("Failed to open camera %d: %s \n", i + 1, cameraList.get(i).name);
      }
      SmartDashboard.putBoolean(cameraList.get(i).name + " OPEN?", cameraList.get(i).isOpen);
    }
    if (LimelightHelpers.getHeartbeat(limelight1) == 0) {
      allOpen = false;
      lime1 = false;
    }
    SmartDashboard.putBoolean("Limelight One Open?", lime1);
    if (LimelightHelpers.getHeartbeat(limelight2) == 0) {
      lime2 = false;
      allOpen = false;
    }
    if(LimelightHelpers.getHeartbeat(limelight3) == 0) {
      lime3 = false;
      allOpen = false;
    }
    SmartDashboard.putBoolean("Limelight Three Open?", lime3);
    SmartDashboard.putBoolean("Limelight Two Open?", lime2);

    SmartDashboard.putNumber("Closest cam", whichClosest());
    SmartDashboard.putNumber("Lime1 Distance", getClosest(2));
    SmartDashboard.putNumber("Lime2 Distance", getClosest(3));
    SmartDashboard.putNumber("Lime3 Distance", getClosest(4));
    SmartDashboard.putNumber("Pi 1 Distance", getClosest(0));
    SmartDashboard.putNumber("Pi 2 Distance", getClosest(1));
  }

  public List<PhotonCamera> getCameras() {
    return cameraList.stream().map(info -> info.camera).collect(Collectors.toList());
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
   * accurate data. Index 0-1 represent photonCameras, 2-3 represent Limelights
   *
   * @return a double representing the ambiguity of the camera
   */
  public double getClosest(int cameraIndex) {
    if (cameraIndex < 0 || cameraIndex > 4) {
      return -1; // Handle invalid camera index
    }

    PhotonPipelineResult result = null;
    if (cameraIndex >= 0 && cameraIndex <= 1) {
      CameraInfo cameraInfo = cameraList.get(cameraIndex);
      result = cameraInfo.camera.getLatestResult();

      if (result == null || !result.hasTargets()) {
        return -1; // Handle the case where no targets are found
      }
    }

    if ((cameraIndex == 2 && !LimelightHelpers.getTV(limelight1))
        || (cameraIndex == 3 && !LimelightHelpers.getTV(limelight2))
        || (cameraIndex == 4 && !LimelightHelpers.getTV(limelight3))) {
      return -1;
    }

    double x, y, z, distance = 0;
    double[] offsets;
    switch (cameraIndex) {
      case 0:
      case 1:
        PhotonTrackedTarget target = result.getBestTarget();
        Transform3d targets = target.getBestCameraToTarget();
        x = targets.getX();
        y = targets.getY();
        z = targets.getZ();
        distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
        return distance;
      case 2:
        offsets = LimelightHelpers.getTargetPose_CameraSpace(limelight1);
        x = offsets[0];
        y = offsets[1];
        z = offsets[2];
        distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
        return distance;
      case 3:
        offsets = LimelightHelpers.getTargetPose_CameraSpace(limelight2);
        x = offsets[0];
        y = offsets[1];
        z = offsets[2];
        distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
        return distance;
      case 4:
        offsets = LimelightHelpers.getTargetPose_CameraSpace(limelight3);
        x = offsets[0];
        y = offsets[1];
        z = offsets[2];
        distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
        return distance;
      default:
        System.out.println("Issue with camera index!");
        break;
    }
    return -1;
  }

  /**
   * Returns the capture time, used for pose estimator
   *
   * @return capture time in milliseconds
   */
  public double getCaptureTime() {
    CameraInfo cameraInfo;
    PhotonPipelineResult result;
    switch (whichClosest()) {
      case 0:
        cameraInfo = cameraList.get(0);
        result = cameraInfo.camera.getLatestResult();
        return result.getTimestampSeconds();
      case 1:
        cameraInfo = cameraList.get(1);
        result = cameraInfo.camera.getLatestResult();
        return result.getTimestampSeconds();
      case 2:
        return Timer.getFPGATimestamp()
                - (LimelightHelpers.getLatency_Capture(limelight1) / 1000.0)
                - (LimelightHelpers.getLatency_Pipeline(limelight1) / 1000.0);
      case 3:
        return Timer.getFPGATimestamp()
                - (LimelightHelpers.getLatency_Capture(limelight2) / 1000.0)
                - (LimelightHelpers.getLatency_Pipeline(limelight2) / 1000.0);
      case 4:
        return Timer.getFPGATimestamp()
                - (LimelightHelpers.getLatency_Capture(limelight3) / 1000.0)
                - (LimelightHelpers.getLatency_Pipeline(limelight3) / 1000.0);
      default:
        return -1;
    }
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
    if(cam2Use == 0 || cam2Use == 1) {
      result = cameraList.get(cam2Use).camera.getLatestResult();
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
    } else if(cam2Use == 2) {
      return LimelightHelpers.getBotPose2d_wpiBlue(limelight1);
    } else if(cam2Use == 3) {
      return LimelightHelpers.getBotPose2d_wpiBlue(limelight2);
    } else if(cam2Use == 4) {
      return LimelightHelpers.getBotPose2d_wpiBlue(limelight3);
    } else {
      return null;
    }
    
    // PhotonPipelineResult result = null;
    // if (camera >= 0 && camera < cameraList.size()) {
    //   result = cameraList.get(camera).camera.getLatestResult();
    // }

    // if (result == null || !result.hasTargets()) {
    //   return null;
    // }

    // Transform3d robotToCamera = cameraList.get(camera).robotToCamera;

    // PhotonTrackedTarget target = result.getBestTarget();
    // if (target.getBestCameraToTarget().getX() > MAX_RANGE) {
    //   return null;
    // }
    // Pose3d robotPose =
    //     PhotonUtils.estimateFieldToRobotAprilTag(
    //         target.getBestCameraToTarget(),
    //         aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(),
    //         robotToCamera.inverse());
    // return robotPose.toPose2d(); temp comment to just use the limelight for testing
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
}
