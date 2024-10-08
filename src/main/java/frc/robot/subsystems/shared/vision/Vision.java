package frc.robot.subsystems.shared.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.Optional;
import lombok.Getter;

public class Vision extends SubsystemBase {
  @Getter private final Camera[] cameras;

  public Vision(Camera... cameras) {
    this.cameras = cameras;
  }

  @Override
  public void periodic() {
    for (Camera camera : cameras) {
      camera.periodic();
    }
  }

  public boolean getValidTarget() {
    boolean isValidTarget = true;
    for (Camera camera : cameras) {
      isValidTarget = camera.getTargetAquired();
    }

    return isValidTarget;
  }

  @SuppressWarnings("unchecked")
  public Optional<Pose3d>[] getPrimaryVisionPoses() {
    return Arrays.stream(cameras)
        .map((Camera camera) -> Optional.of(camera.getPrimaryPose()))
        .toArray(Optional[]::new);
  }

  @SuppressWarnings("unchecked")
  public Optional<Pose3d>[] getSecondaryVisionPoses() {
    return Arrays.stream(cameras)
        .map((Camera camera) -> Optional.of(camera.getSecondaryPose()))
        .toArray(Optional[]::new);
  }

  public double[] getFrameTimestamps() {
    return Arrays.stream(cameras)
        .mapToDouble((Camera camera) -> camera.getFrameTimestamp())
        .toArray();
  }
}
