package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class SnapbackMechanism3d {
  public static final double EXTENSION_0_HEIGHT = 0.099;
  public static final double EXTENSION_1_MAX = 0.4;
  public static final double EXTENSION_2_MAX = 0.4;

  public static final double MIN_EXTENSION = 0.0;
  public static final double MAX_EXTENSION = MIN_EXTENSION + EXTENSION_1_MAX + EXTENSION_2_MAX;

  private SnapbackMechanism3d() {}

  public static final Pose3d[] getPoses(
      boolean intakeDeployed,
      Rotation2d hoodAngle,
      double leftExtensionMeters,
      double rightExtensionMeters) {

    Pose3d hoodPose =
        new Pose3d(
            -0.28,
            0.0,
            0.48,
            new Rotation3d(0.0, -hoodAngle.getRadians() + Units.degreesToRadians(38.0), 0.0));

    Pose3d intakeTopBarPose = new Pose3d();
    Pose3d intakeBottomBarPose = new Pose3d();
    Pose3d intakeRollersPose = new Pose3d();

    if (!intakeDeployed) {
      intakeTopBarPose =
          new Pose3d(0.11, 0.0, 0.2635, new Rotation3d(0.0, -Units.degreesToRadians(65.0), 0.0));
      intakeBottomBarPose =
          new Pose3d(0.123, 0.0, 0.114, new Rotation3d(0.0, -Units.degreesToRadians(65.0), 0.0));
      intakeRollersPose =
          new Pose3d(
              -0.005 - intakeTopBarPose.getX(),
              0.0,
              0.375 + intakeTopBarPose.getZ(),
              new Rotation3d(0.0, Units.degreesToRadians(50.0), 0.0));
    } else {
      intakeTopBarPose = new Pose3d(0.11, 0.0, 0.2635, new Rotation3d());
      intakeBottomBarPose = new Pose3d(0.123, 0.0, 0.114, new Rotation3d());
      intakeRollersPose = new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
    }

    leftExtensionMeters = MathUtil.clamp(leftExtensionMeters, MIN_EXTENSION, MAX_EXTENSION);
    double leftExtensionFraction =
        MathUtil.inverseInterpolate(MIN_EXTENSION, MAX_EXTENSION, leftExtensionMeters);

    Pose3d leftExtension0Pose = new Pose3d(0.05, -0.21, EXTENSION_0_HEIGHT, new Rotation3d());

    Pose3d leftExtension1Pose =
        leftExtension0Pose.transformBy(
            new Transform3d(0.0, 0.0, EXTENSION_1_MAX * leftExtensionFraction, new Rotation3d()));

    Pose3d leftExtension2Pose =
        leftExtension1Pose.transformBy(
            new Transform3d(0.0, 0.0, EXTENSION_2_MAX * leftExtensionFraction, new Rotation3d()));

    rightExtensionMeters = MathUtil.clamp(leftExtensionMeters, MIN_EXTENSION, MAX_EXTENSION);
    double rightExtensionFraction =
        MathUtil.inverseInterpolate(MIN_EXTENSION, MAX_EXTENSION, leftExtensionMeters);

    Pose3d rightExtension0Pose = new Pose3d(0.05, 0.21, EXTENSION_0_HEIGHT, new Rotation3d());

    Pose3d rightExtension1Pose =
        rightExtension0Pose.transformBy(
            new Transform3d(0.0, 0.0, EXTENSION_1_MAX * rightExtensionFraction, new Rotation3d()));

    Pose3d rightExtension2Pose =
        rightExtension1Pose.transformBy(
            new Transform3d(0.0, 0.0, EXTENSION_2_MAX * rightExtensionFraction, new Rotation3d()));

    return new Pose3d[] {
      new Pose3d(),
      new Pose3d(),
      new Pose3d(),
      hoodPose,
      intakeTopBarPose,
      intakeBottomBarPose,
      intakeRollersPose,
      rightExtension1Pose,
      rightExtension2Pose,
      leftExtension1Pose,
      leftExtension2Pose
    };
  }
}
