package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
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

  private final TimeInterpolatableBuffer<Pose2d> robotPoseBuffer =
      TimeInterpolatableBuffer.createBuffer(0.5);
  private Supplier<Pose2d> drivePoseSupplier = null;

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

    Pose2d robotPose = drivePoseSupplier.get();
    robotPoseBuffer.addSample(Timer.getFPGATimestamp(), robotPose);

    Optional<Rotation2d> targetAngle = getTargetGyroAngle();
    Optional<Double> distance = getSpeakerDistance();
    Optional<Pose2d> calculatedRobotPose = getRobotPose();
    if (targetAngle.isPresent()) {
      Logger.recordOutput(name + "/TargetAngle", targetAngle.get());
    }
    if (distance.isPresent()) {
      Logger.recordOutput(name + "/SpeakerDistance", distance.get());
    }
    if (calculatedRobotPose.isPresent()) {
      Logger.recordOutput(name + "/CalculatedRobotPose", calculatedRobotPose.get());
    }
  }

  public Optional<Rotation2d> getTargetGyroAngle() {
    Optional<Pose2d> robotPose = robotPoseBuffer.getSample(inputs.timeStamp);
    if (robotPose.isPresent() && inputs.tv) {
      return Optional.of(robotPose.get().getRotation().minus(inputs.tx));
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

  public void setDrivePoseSupplier(Supplier<Pose2d> drivePoseSupplier) {
    this.drivePoseSupplier = drivePoseSupplier;
  }

  public Optional<Pose2d> getRobotPose() {
    if (robotPoseBuffer.getSample(inputs.timeStamp).isPresent() && inputs.tv) {
      Pose2d currentPoseFromDrive = drivePoseSupplier.get();
      Pose2d capturePoseFromDrive = robotPoseBuffer.getSample(inputs.timeStamp).get();
      Pose2d capturePoseFromCam = inputs.robotPose.toPose2d();

      Pose2d currentPoseFromCam =
          capturePoseFromCam.plus(currentPoseFromDrive.minus(capturePoseFromDrive));
      return Optional.of(currentPoseFromCam);
    }
    return Optional.empty();
  }
}
