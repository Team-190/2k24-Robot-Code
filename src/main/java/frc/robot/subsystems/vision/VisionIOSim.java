package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.FieldConstants;
import java.util.function.Supplier;

public class VisionIOSim implements VisionIO {
  private final Supplier<Pose2d> poseSupplier;

  private final Transform3d cameraTransform;
  private final Pose3d[] targetPoses;
  private static final Rotation2d fieldOfViewHorizontal = Rotation2d.fromDegrees(63.3);
  private static final Rotation2d fieldOfViewVertical = Rotation2d.fromDegrees(49.7);

  public VisionIOSim(VisionMode llMode, Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;

    switch (llMode) {
      case AprilTags:
        cameraTransform = new Transform3d(0.3, 0, 0.1, new Rotation3d(0, -Math.PI / 4.0, 0));
        targetPoses =
            new Pose3d[] {
              FieldConstants.aprilTags.getTagPose(4).get(),
              FieldConstants.aprilTags.getTagPose(7).get()
            };
        break;
      case Notes:
        cameraTransform =
            new Transform3d(-0.2, 0, 0.8, new Rotation3d(0, Units.degreesToRadians(20), Math.PI));
        targetPoses = new Pose3d[11];
        for (int i = 0; i < 5; i++) {
          targetPoses[i] =
              new Pose3d(
                  new Pose2d(
                      FieldConstants.StagingLocations.centerlineTranslations[i], new Rotation2d()));
        }
        for (int i = 5; i < 8; i++) {
          targetPoses[i] =
              new Pose3d(
                  new Pose2d(
                      FieldConstants.StagingLocations.spikeTranslations[i - 5], new Rotation2d()));
        }
        for (int i = 8; i < 11; i++) {
          Translation2d gamePieceTranslation =
              new Translation2d(
                  FieldConstants.fieldLength
                      - FieldConstants.StagingLocations.spikeTranslations[i - 8].getX(),
                  FieldConstants.StagingLocations.spikeTranslations[i - 8].getY());
          targetPoses[i] = new Pose3d(new Pose2d(gamePieceTranslation, new Rotation2d()));
        }
        break;
      default:
        cameraTransform = new Transform3d();
        targetPoses = new Pose3d[0];
        break;
    }
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(VisionIOInputs inputs) {
    Pose3d cameraPose = new Pose3d(poseSupplier.get()).transformBy(cameraTransform);
    Double closestNorm = null;
    Rotation2d tx = null;
    Rotation2d ty = null;
    Pose3d targetRelativePose;

    for (int i = 0; i < targetPoses.length; i++) {
      targetRelativePose = targetPoses[i].relativeTo(cameraPose);
      Double currentNorm = targetRelativePose.getTranslation().getNorm();

      double xyAngle = Math.atan(targetRelativePose.getY() / targetRelativePose.getX());
      double xzAngle = Math.atan(targetRelativePose.getZ() / targetRelativePose.getX());

      boolean isValid =
          (Math.abs(xyAngle) < fieldOfViewHorizontal.getRadians() / 2.0)
              && (Math.abs(xzAngle) < fieldOfViewVertical.getRadians() / 2.0);

      if (isValid && (closestNorm == null || currentNorm < closestNorm)) {
        tx = new Rotation2d(-xyAngle);
        ty = new Rotation2d(xzAngle);
        closestNorm = currentNorm;
      }
    }

    inputs.timeStamp = Timer.getFPGATimestamp();
    inputs.tv = closestNorm != null;
    if (inputs.tv) {
      inputs.tx = tx;
      inputs.ty = ty;
    }
  }
}
