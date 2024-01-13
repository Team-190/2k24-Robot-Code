package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs;

  private final TimeInterpolatableBuffer<Rotation2d> gyroBuffer =
      TimeInterpolatableBuffer.createBuffer(0.5);
  private Supplier<Rotation2d> gyroSupplier = null;

  public Vision(VisionIO io) {
    this.io = io;
    inputs = new VisionIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);

    Rotation2d gyroPosition = gyroSupplier.get();
    gyroBuffer.addSample(Timer.getFPGATimestamp(), gyroPosition);
  }

  public Optional<Rotation2d> getTargetGyroAngle() {
    Optional<Rotation2d> gyroPosition = gyroBuffer.getSample(inputs.timeStamp);
    if (gyroPosition.isPresent() && inputs.tv) {
      return Optional.of(gyroPosition.get().plus(inputs.tx));
    } else {
      return Optional.empty();
    }
  }

  public void setGyroSupplier(Supplier<Rotation2d> gyroSupplier) {
    this.gyroSupplier = gyroSupplier;
  }
}
