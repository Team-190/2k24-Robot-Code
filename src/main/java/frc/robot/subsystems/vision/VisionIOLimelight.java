package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.FieldConstants;

public class VisionIOLimelight implements VisionIO {
  private final NetworkTable table;
  private final DoubleSubscriber tx;
  private final DoubleSubscriber ty;
  private final DoubleSubscriber tv;
  private final DoubleArraySubscriber botpose;

  public VisionIOLimelight(VisionMode mode) {
    table = NetworkTableInstance.getDefault().getTable(mode.name);
    tx = table.getDoubleTopic("tx").subscribe(0.0);
    ty = table.getDoubleTopic("ty").subscribe(0.0);
    tv = table.getDoubleTopic("tv").subscribe(0.0);
    botpose =
        table
            .getDoubleArrayTopic("botpose")
            .subscribe(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.timeStamp =
        botpose.getLastChange() * 0.000001
            - botpose.get()[6]
                * 0.001; // Calculate the time (in seconds) when the Limelight captured the frame

    inputs.tx = Rotation2d.fromDegrees(tx.get());
    inputs.ty = Rotation2d.fromDegrees(ty.get());
    inputs.tv = tv.get() != 0;
    inputs.robotPose =
        new Pose3d(
            botpose.get()[0] + FieldConstants.fieldLength / 2.0,
            botpose.get()[1] + FieldConstants.fieldWidth / 2.0,
            botpose.get()[2],
            new Rotation3d(
                Units.degreesToRadians(botpose.get()[3]),
                Units.degreesToRadians(botpose.get()[4]),
                Units.degreesToRadians(botpose.get()[5])));
  }

  @Override
  public boolean getTv() {
    return tv.get() != 0;
  }

  @Override
  public void enableLEDs() {
    table.getEntry("ledMode").setNumber(3);
  }

  @Override
  public void disableLEDs() {
    table.getEntry("ledMode").setNumber(1);
  }
}
