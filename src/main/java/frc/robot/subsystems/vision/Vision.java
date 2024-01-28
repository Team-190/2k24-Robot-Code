package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.VirtualSubsystem;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends VirtualSubsystem {
  private final String name;
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs;
  private final double CAMERA_OFFSET = Units.inchesToMeters(14);
  private final double SPEAKER_TAG_HEIGHT = Units.inchesToMeters(57.13);
  private final double CAMERA_HEIGHT = Units.inchesToMeters(8.75);
  private final LoggedTunableNumber CAMERA_ANGLE = new LoggedTunableNumber("Vision/CameraAngle");

  private final TimeInterpolatableBuffer<Rotation2d> gyroBuffer =
      TimeInterpolatableBuffer.createBuffer(0.5);
  private Supplier<Rotation2d> gyroSupplier = null;

  public Vision(String name, VisionIO io) {
    this.name = name;
    this.io = io;
    inputs = new VisionIOInputsAutoLogged();
    CAMERA_ANGLE.initDefault(0.6109);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    Rotation2d gyroPosition = gyroSupplier.get();
    gyroBuffer.addSample(Timer.getFPGATimestamp(), gyroPosition);
    Optional<Rotation2d> targetAngle = getTargetGyroAngle();
    Optional<Double> distance = getSpeakerDistance();
    if (targetAngle.isPresent()) {
      Logger.recordOutput(name + "/TargetAngle", targetAngle.get());
    }
    if (distance.isPresent()) {
      Logger.recordOutput(name + "/SpeakerDistance", distance.get());
    }
  }

  public Optional<Rotation2d> getTargetGyroAngle() {
    Optional<Rotation2d> gyroPosition = gyroBuffer.getSample(inputs.timeStamp);
    if (gyroPosition.isPresent() && inputs.tv) {
      return Optional.of(gyroPosition.get().minus(inputs.tx));
    } else {
      return Optional.empty();
    }
  }

  public Optional<Double> getSpeakerDistance() {
    if (inputs.tv) {
      return Optional.of(
          (SPEAKER_TAG_HEIGHT - CAMERA_HEIGHT)
                  / Math.tan(CAMERA_ANGLE.get() + inputs.ty.getRadians())
              + CAMERA_OFFSET);
    }
    return Optional.empty();
  }

  public void setGyroSupplier(Supplier<Rotation2d> gyroSupplier) {
    this.gyroSupplier = gyroSupplier;
  }
}
