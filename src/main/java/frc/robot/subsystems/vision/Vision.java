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
  private final double CAMERA_OFFSET = Units.inchesToMeters(12.75);
  private final double SPEAKER_TAG_HEIGHT = Units.inchesToMeters(57.13);
  private final double CAMERA_HEIGHT = Units.inchesToMeters(18);
  private final LoggedTunableNumber CAMERA_ANGLE = new LoggedTunableNumber("Vision/Camera Angle");
  private double lastValidTimeStamp = Double.NEGATIVE_INFINITY;
  private Pose2d lastValidRobotPose = new Pose2d();
  private static final double BUFFER_SECONDS = 3;

  private final TimeInterpolatableBuffer<Pose2d> robotPoseBuffer =
      TimeInterpolatableBuffer.createBuffer(BUFFER_SECONDS);
  private Supplier<Pose2d> drivePoseSupplier = null;

  public Vision(String name, VisionIO io) {
    this.name = name;
    this.io = io;
    inputs = new VisionIOInputsAutoLogged();
    CAMERA_ANGLE.initDefault(Units.degreesToRadians(33.0));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    if (inputs.tv) {
      lastValidTimeStamp = inputs.timeStamp;
      lastValidRobotPose = inputs.robotPose.toPose2d();
    }
    Pose2d robotPose = drivePoseSupplier.get();
    robotPoseBuffer.addSample(Timer.getFPGATimestamp(), robotPose);

    Optional<Rotation2d> targetAngle = getTargetGyroAngle();
    Optional<Double> distance = getSpeakerDistance();
    Optional<Pose2d> calculatedRobotPose = getRobotPose();
    if (targetAngle.isPresent()) {
      Logger.recordOutput(name + "/Target Angle", targetAngle.get());
    }
    if (distance.isPresent()) {
      Logger.recordOutput(name + "/Speaker Distance", distance.get());
    }
    if (calculatedRobotPose.isPresent()) {
      Logger.recordOutput(name + "/Calculated Robot Pose", calculatedRobotPose.get());
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
    if ((Timer.getFPGATimestamp() - lastValidTimeStamp) <= BUFFER_SECONDS
        && robotPoseBuffer.getSample(lastValidTimeStamp).isPresent()) {
      Pose2d currentPoseFromDrive = drivePoseSupplier.get();
      Pose2d capturePoseFromDrive = robotPoseBuffer.getSample(lastValidTimeStamp).get();
      Pose2d capturePoseFromCam = lastValidRobotPose;

      Pose2d currentPoseFromCam =
          capturePoseFromCam.plus(currentPoseFromDrive.minus(capturePoseFromDrive));
      return Optional.of(currentPoseFromCam);
    }
    return Optional.empty();
  }
}
