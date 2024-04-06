package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public Rotation2d tx = new Rotation2d();
    public Rotation2d ty = new Rotation2d();
    public boolean tv = false;
    public double timeStamp = 0.0;
    public Pose3d robotPose = new Pose3d();
    public long pipeline = 0;
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default boolean getTv() {
    return false;
  }

  public default void enableLEDs() {}

  public default void disableLEDs() {}

  public default void setPipeline(double pipeline) {}
}
