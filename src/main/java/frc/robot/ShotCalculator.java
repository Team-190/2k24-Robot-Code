package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class ShotCalculator {
  private static final InterpolatingDoubleTreeMap shooterSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap shooterAngleMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap flightTimeMap = new InterpolatingDoubleTreeMap();

  private static final LoggedTunableNumber SHOOTER_SPEED_TOLERANCE =
      new LoggedTunableNumber("ShotCalculator/Shooter Speed Tolerance");

  private static final LoggedTunableNumber HOOD_ANGLE_TOLERANCE =
      new LoggedTunableNumber("ShotCalculator/Hood Angle Tolerance");

  static {
    // Units: radians/second
    shooterSpeedMap.put(Units.feetToMeters(7.34), 800.0);
    shooterSpeedMap.put(Units.feetToMeters(8.2), 800.0);
    shooterSpeedMap.put(Units.feetToMeters(9.0), 800.0);
    shooterSpeedMap.put(Units.feetToMeters(10.0), 800.0);
    shooterSpeedMap.put(Units.feetToMeters(11.0), 800.0); // 600
    shooterSpeedMap.put(Units.feetToMeters(12.2), 800.0);
    shooterSpeedMap.put(Units.feetToMeters(14.25), 800.0);

    // Units: radians
    shooterAngleMap.put(Units.feetToMeters(7.34), 0.0);
    shooterAngleMap.put(Units.feetToMeters(8.2), 0.0);
    shooterAngleMap.put(Units.feetToMeters(9.0), 0.27);
    shooterAngleMap.put(Units.feetToMeters(10.0), 0.35);
    shooterAngleMap.put(Units.feetToMeters(11.0), 0.43); // 0.3
    shooterAngleMap.put(Units.feetToMeters(12.2), 0.44); // 0.36
    shooterAngleMap.put(Units.feetToMeters(14.25), 0.45); // 0.376

    // Units: seconds
    flightTimeMap.put(0.0, 0.179);
    flightTimeMap.put(3.5, 0.294);

    SHOOTER_SPEED_TOLERANCE.initDefault(40.0);
    HOOD_ANGLE_TOLERANCE.initDefault(0.017);
  }

  public static AimingParameters poseCalculation(
      Translation2d fieldRelativePose, Translation2d fieldRelativeVelocity) {
    Translation2d speakerPose =
        AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.getTranslation());
    double distanceToSpeaker = fieldRelativePose.getDistance(speakerPose);
    Translation2d effectiveAimingPose =
        fieldRelativePose.plus(fieldRelativeVelocity.times(flightTimeMap.get(distanceToSpeaker)));
    double effectiveDistanceToSpeaker = effectiveAimingPose.getDistance(speakerPose);

    Rotation2d setpointAngle = speakerPose.minus(effectiveAimingPose).getAngle();
    double tangentialVelocity = -fieldRelativeVelocity.rotateBy(setpointAngle.unaryMinus()).getY();
    double radialVelocity = tangentialVelocity / effectiveDistanceToSpeaker;
    Logger.recordOutput("ShotCalculator/effectiveDistanceToSpeaker", effectiveDistanceToSpeaker);
    Logger.recordOutput(
        "ShotCalculator/effectiveAimingPose", new Pose2d(effectiveAimingPose, new Rotation2d()));
    Logger.recordOutput("ShotCalculator/robotAngle", setpointAngle);
    return new AimingParameters(
        setpointAngle,
        radialVelocity,
        shooterSpeedMap.get(effectiveDistanceToSpeaker),
        new Rotation2d(shooterAngleMap.get(effectiveDistanceToSpeaker)));
  }

  public static AimingParameters angleCalculation() {
    return null;
  }

  public static boolean shooterReady(
      Drive drive, Hood hood, Shooter shooter, Vision aprilTagVision) {
    if (aprilTagVision.getRobotPose().isPresent()) {
      AimingParameters setpoints =
          poseCalculation(
              aprilTagVision.getRobotPose().get().getTranslation(),
              drive.getFieldRelativeVelocity());
      return (Math.abs(hood.getPosition().getRadians() - setpoints.shooterAngle.getRadians())
              <= HOOD_ANGLE_TOLERANCE.get())
          && (Math.abs(shooter.getSpeed() - setpoints.shooterSpeed)
              <= SHOOTER_SPEED_TOLERANCE.get());
    }
    return false;
  }

  public static record AimingParameters(
      Rotation2d robotAngle, double radialVelocity, double shooterSpeed, Rotation2d shooterAngle) {}
}
