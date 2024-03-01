package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class SnapbackMechanism3d {
  public static final double EXTENSION_0_HEIGHT = 0.0;
  public static final double EXTENSION_1_MAX = 0.0;
  public static final double EXTENSION_2_MAX = 0.0;

  public static final double MIN_EXTENSION = 0.0;
  public static final double MAX_EXTENSION = MIN_EXTENSION + EXTENSION_1_MAX + EXTENSION_2_MAX;

  private SnapbackMechanism3d() {}

  public static final Pose3d[] getPoses(
      boolean intakeDeployed, Rotation2d hoodAngle, Rotation2d ampAngle, double extensionMeters) {

    Pose3d intakePose = null;
    if (intakeDeployed) {
      intakePose = null;
    } else {
      intakePose = null;
    }

    Pose3d hoodPose = new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, hoodAngle.getRadians(), 0.0));
    Pose3d ampPose = new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, ampAngle.getRadians(), 0.0));

    extensionMeters = MathUtil.clamp(extensionMeters, MIN_EXTENSION, MAX_EXTENSION);
    double extensionFraction =
        MathUtil.inverseInterpolate(MIN_EXTENSION, MAX_EXTENSION, extensionMeters);

    Pose3d extension0Pose = new Pose3d(0.0, 0.0, EXTENSION_0_HEIGHT, new Rotation3d());

    Pose3d extension1Pose =
        extension0Pose.transformBy(
            new Transform3d(EXTENSION_1_MAX * extensionFraction, 0.0, 0.0, new Rotation3d()));

    Pose3d extension2Pose =
        extension1Pose.transformBy(
            new Transform3d(EXTENSION_2_MAX * extensionFraction, 0.0, 0.0, new Rotation3d()));
    return new Pose3d[] {
      intakePose, hoodPose, ampPose, extension0Pose, extension1Pose, extension2Pose
    };
  }
}
