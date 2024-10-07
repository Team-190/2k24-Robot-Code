package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.Optional;
import lombok.Getter;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class CameraIOPhotonVision implements CameraIO {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator primaryPhotonPoseEstimator;
  private final PhotonPoseEstimator secondaryPhotonPoseEstimator;

  @Getter private final CameraType cameraType;
  @Getter private final double horizontalFOV;
  @Getter private final double verticalFOV;
  @Getter private final double primaryXYStandardDeviationCoefficient;
  @Getter private final double secondaryXYStandardDeviationCoefficient;

  @Getter private Rotation2d xOffset;
  @Getter private Rotation2d yOffset;
  @Getter private boolean targetAquired;
  @Getter private int totalTargets;
  @Getter private double averageDistance;
  @Getter private double frameTimestamp;
  @Getter private Pose3d primaryPose;
  @Getter Pose3d secondaryPose;

  public CameraIOPhotonVision(
      String cameraName,
      CameraType cameraType,
      AprilTagFieldLayout aprilTagFieldLayout,
      Transform3d robotToCamera,
      PoseStrategy primaryStrategy,
      PoseStrategy secondaryStrategy) {
    camera = new PhotonCamera(cameraName);
    this.cameraType = cameraType;
    this.primaryPhotonPoseEstimator =
        new PhotonPoseEstimator(aprilTagFieldLayout, primaryStrategy, camera, robotToCamera);
    this.secondaryPhotonPoseEstimator =
        new PhotonPoseEstimator(aprilTagFieldLayout, secondaryStrategy, camera, robotToCamera);
    this.horizontalFOV = cameraType.horizontalFOV;
    this.verticalFOV = cameraType.verticalFOV;
    this.primaryXYStandardDeviationCoefficient = cameraType.primaryXYStandardDeviationCoefficient;
    this.secondaryXYStandardDeviationCoefficient =
        cameraType.secondaryXYStandardDeviationCoefficient;
  }

  @Override
  public void updateInputs(CameraIOInputs inputs) {
    PhotonPipelineResult result = camera.getLatestResult();
    if (result.hasTargets()) {
      primaryPhotonPoseEstimator.setReferencePose(inputs.primaryPose);
      inputs.targetAquired = true;
      inputs.totalTargets = result.getTargets().size();
      inputs.frameTimestamp = result.getTimestampSeconds();
      inputs.xOffset = Rotation2d.fromDegrees(result.getBestTarget().getYaw());
      inputs.yOffset = Rotation2d.fromDegrees(result.getBestTarget().getPitch());
      Optional<EstimatedRobotPose> primaryEstimatedPose = primaryPhotonPoseEstimator.update(result);
      Optional<EstimatedRobotPose> secondaryEstimatedPose =
          secondaryPhotonPoseEstimator.update(result);
      if (primaryEstimatedPose.isPresent()) {
        inputs.primaryPose = new Pose3d(primaryEstimatedPose.get().estimatedPose.toPose2d());
      }
      if (secondaryEstimatedPose.isPresent()) {
        inputs.secondaryPose = new Pose3d(secondaryEstimatedPose.get().estimatedPose.toPose2d());
      }
    }

    xOffset = inputs.xOffset;
    yOffset = inputs.yOffset;
    targetAquired = inputs.targetAquired;
    totalTargets = inputs.totalTargets;
    averageDistance = inputs.averageDistance;
    frameTimestamp = inputs.frameTimestamp;
    primaryPose = inputs.primaryPose;
    secondaryPose = inputs.secondaryPose;
  }

  @Override
  public void setPipeline(int pipeline) {
    camera.setPipelineIndex(pipeline);
  }

  @Override
  public String toString() {
    return "photonvision" + "-" + camera.getName();
  }
}
