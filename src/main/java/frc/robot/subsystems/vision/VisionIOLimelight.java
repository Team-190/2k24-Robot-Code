package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionIOLimelight implements VisionIO {
  private final DoubleSubscriber tx;
  private final DoubleSubscriber ty;
  private final DoubleSubscriber tl;
  private final DoubleSubscriber tv;

  public VisionIOLimelight(VisionMode mode) {
    var table = NetworkTableInstance.getDefault().getTable(mode.name);
    tx = table.getDoubleTopic("tx").subscribe(0.0);
    ty = table.getDoubleTopic("ty").subscribe(0.0);
    tl = table.getDoubleTopic("tl").subscribe(0.0);
    tv = table.getDoubleTopic("tv").subscribe(0.0);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.timeStamp =
        tl.getLastChange() * 0.000001
            - tl.get()
                * 0.001; // Calculate the time (in seconds) when the Limelight captured the frame
    inputs.tx = Rotation2d.fromDegrees(tx.get());
    inputs.ty = Rotation2d.fromDegrees(ty.get());
    inputs.tv = tv.get() != 0;
  }
}
