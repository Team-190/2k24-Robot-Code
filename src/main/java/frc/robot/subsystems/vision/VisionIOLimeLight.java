package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionIOLimeLight implements VisionIO {
  private final DoubleSubscriber tx =
      NetworkTableInstance.getDefault().getTable("limelight").getDoubleTopic("tx").subscribe(0.0);
  private final DoubleSubscriber ty =
      NetworkTableInstance.getDefault().getTable("limelight").getDoubleTopic("ty").subscribe(0.0);
  private final DoubleSubscriber tl =
      NetworkTableInstance.getDefault().getTable("limelight").getDoubleTopic("tl").subscribe(0.0);
  private final DoubleSubscriber tv =
      NetworkTableInstance.getDefault().getTable("limelight").getDoubleTopic("tv").subscribe(0.0);

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.timeStamp =
        tl.getLastChange() * 0.000001
            - tl.get()
                * 0.001; // calculate the time (in seconds) when the limelight captured the frame
    inputs.tx = Rotation2d.fromDegrees(tx.get());
    inputs.ty = Rotation2d.fromDegrees(ty.get());
    inputs.tv = tv.get() != 0;
  }
}
