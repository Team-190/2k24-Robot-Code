package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.drive.DriveConstants;
import frc.robot.subsystems.vision.Camera;
import frc.robot.util.AllianceFlipUtil;
import java.util.Optional;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  private static final InterpolatingDoubleTreeMap speakerShotSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap feedShotSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap speakerShotAngleMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap feedShotAngleMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  @Getter
  private static ControlData controlData =
      new ControlData(
          new Rotation2d(), 0.0, new Rotation2d(), new Rotation2d(), 0.0, new Rotation2d());

  @Getter @Setter private static double speakerFlywheelCompensation = 0.0;
  @Getter @Setter private static double speakerAngleCompensation = 0.0;

  private static SwerveDrivePoseEstimator poseEstimator;

  private static Rotation2d robotHeading;
  private static SwerveModulePosition[] modulePositions;

  static {
    // Units: radians per second
    speakerShotSpeedMap.put(0.0, 0.0);

    // Units: radians per second
    feedShotSpeedMap.put(0.0, 0.0);

    // Units: radians
    speakerShotAngleMap.put(0.0, 0.0);

    // Units: radians
    feedShotAngleMap.put(0.0, 0.0);

    // Units: seconds
    timeOfFlightMap.put(0.0, 0.0);

    modulePositions = new SwerveModulePosition[4];

    for (int i = 0; i < modulePositions.length; i++) {
      modulePositions[i] = new SwerveModulePosition();
    }

    poseEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.KINEMATICS, new Rotation2d(), modulePositions, new Pose2d());
  }

  public RobotState() {}

  public static void periodic(
      Rotation2d robotHeading,
      double robotYawVelocity,
      Translation2d robotFieldRelativeVelocity,
      SwerveModulePosition[] modulePositions,
      Camera[] cameras,
      boolean targetAquired,
      Optional<Pose3d>[] visionPrimaryPoses,
      Optional<Pose3d>[] visionSecondaryPoses,
      double[] visionFrameTimestamps) {

    RobotState.robotHeading = robotHeading;
    RobotState.modulePositions = modulePositions;

    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), robotHeading, modulePositions);

    if (targetAquired && robotYawVelocity < Units.degreesToRadians(720.0)) {
      for (int i = 0; i < visionPrimaryPoses.length; i++) {
        if (visionPrimaryPoses[i].isPresent()) {
          double xyStddev =
              cameras[i].getPrimaryXYStandardDeviationCoefficient()
                  * Math.pow(cameras[i].getAverageDistance(), 2.0)
                  / cameras[i].getTotalTargets()
                  * cameras[i].getHorizontalFOV();
          poseEstimator.addVisionMeasurement(
              visionPrimaryPoses[i].get().toPose2d(),
              visionFrameTimestamps[i],
              VecBuilder.fill(xyStddev, xyStddev, Double.POSITIVE_INFINITY));
        }
      }
      for (int i = 0; i < visionSecondaryPoses.length; i++) {
        if (visionSecondaryPoses[i].isPresent()) {
          double xyStddev =
              cameras[i].getSecondaryXYStandardDeviationCoefficient()
                  * Math.pow(cameras[i].getAverageDistance(), 2.0)
                  / cameras[i].getTotalTargets()
                  * cameras[i].getHorizontalFOV();
          poseEstimator.addVisionMeasurement(
              visionSecondaryPoses[i].get().toPose2d(),
              visionFrameTimestamps[i],
              VecBuilder.fill(xyStddev, xyStddev, Double.POSITIVE_INFINITY));
        }
      }
    }

    Translation2d speakerPose =
        AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d());
    Translation2d ampPose = AllianceFlipUtil.apply(FieldConstants.ampCenter);
    double distanceToSpeaker =
        poseEstimator.getEstimatedPosition().getTranslation().getDistance(speakerPose);
    double distanceToAmp =
        poseEstimator.getEstimatedPosition().getTranslation().getDistance(ampPose);
    Translation2d effectiveSpeakerAimingPose =
        poseEstimator
            .getEstimatedPosition()
            .getTranslation()
            .plus(robotFieldRelativeVelocity.times(timeOfFlightMap.get(distanceToSpeaker)));
    Translation2d effectiveFeedAmpAimingPose =
        poseEstimator
            .getEstimatedPosition()
            .getTranslation()
            .plus(robotFieldRelativeVelocity.times(timeOfFlightMap.get(distanceToAmp)));
    double effectiveDistanceToSpeaker = effectiveSpeakerAimingPose.getDistance(speakerPose);
    double effectiveDistanceToAmp = effectiveFeedAmpAimingPose.getDistance(ampPose);

    Rotation2d speakerRobotAngle =
        speakerPose.minus(effectiveSpeakerAimingPose).getAngle().minus(robotHeading);
    Rotation2d feedAmpRobotAngle =
        ampPose.minus(effectiveFeedAmpAimingPose).getAngle().minus(robotHeading);
    controlData =
        new ControlData(
            speakerRobotAngle,
            speakerShotSpeedMap.get(effectiveDistanceToSpeaker),
            new Rotation2d(speakerShotAngleMap.get(effectiveDistanceToSpeaker)),
            feedAmpRobotAngle,
            feedShotSpeedMap.get(effectiveDistanceToAmp),
            new Rotation2d(feedShotAngleMap.get(effectiveDistanceToAmp)));

    Logger.recordOutput(
        "RobotState/Pose Data/Estimated Pose", poseEstimator.getEstimatedPosition());
    Logger.recordOutput(
        "RobotState/Pose Data/Effective Speaker Aiming Pose",
        new Pose2d(effectiveSpeakerAimingPose, new Rotation2d()));
    Logger.recordOutput(
        "RobotState/Pose Data/Effective Feed Aiming Pose",
        new Pose2d(effectiveFeedAmpAimingPose, new Rotation2d()));
    Logger.recordOutput(
        "RobotState/Pose Data/Effective Distance To Speaker", effectiveDistanceToSpeaker);
    Logger.recordOutput("RobotState/Pose Data/Effective Distance To Amp", effectiveDistanceToAmp);
    Logger.recordOutput(
        "RobotState/Signal Data/Rio Bus Utilization",
        RobotController.getCANStatus().percentBusUtilization);
    Logger.recordOutput(
        "RobotState/Signal Data/CANivore Bus Utilization",
        CANBus.getStatus(DriveConstants.CANIVORE).BusUtilization);
    Logger.recordOutput(
        "RobotState/ControlData/Speaker Robot Angle", controlData.speakerRobotAngle());
    Logger.recordOutput("RobotState/ControlData/Feed Robot Angle", controlData.feedRobotAngle());
    Logger.recordOutput(
        "RobotState/ControlData/Speaker Shot Speed", controlData.speakerShotSpeed());
    Logger.recordOutput("RobotState/ControlData/Speaker Arm Angle", controlData.speakerArmAngle());
    Logger.recordOutput("RobotState/ControlData/Feed Shot Speed", controlData.feedShotSpeed());
    Logger.recordOutput("RobotState/ControlData/Feed Arm Angle", controlData.feedArmAngle());
  }

  public static Pose2d getRobotPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public static void resetRobotPose(Pose2d pose) {
    poseEstimator.resetPosition(robotHeading, modulePositions, pose);
  }

  public static record ControlData(
      Rotation2d speakerRobotAngle,
      double speakerShotSpeed,
      Rotation2d speakerArmAngle,
      Rotation2d feedRobotAngle,
      double feedShotSpeed,
      Rotation2d feedArmAngle) {}
}
