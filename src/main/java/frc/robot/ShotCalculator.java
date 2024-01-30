package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

public class ShotCalculator {
  private static final InterpolatingDoubleTreeMap shooterSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap shooterAngleMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap flightTimeMap = new InterpolatingDoubleTreeMap();

  static {
    // Units: radians/second
    shooterSpeedMap.put(Units.inchesToMeters(60), 10.0);
    shooterSpeedMap.put(Units.inchesToMeters(80), 17.0);
    shooterSpeedMap.put(Units.inchesToMeters(100), 9.0);
    shooterSpeedMap.put(Units.inchesToMeters(120), 30.0);
    shooterSpeedMap.put(Units.inchesToMeters(140), 19.0);
    shooterSpeedMap.put(Units.inchesToMeters(160), 88.0);
    shooterSpeedMap.put(Units.inchesToMeters(180), 10.0);

    // Units: radians
    shooterAngleMap.put(Units.inchesToMeters(60), 1.0);
    shooterAngleMap.put(Units.inchesToMeters(80), 0.0);
    shooterAngleMap.put(Units.inchesToMeters(100), 3.5);
    shooterAngleMap.put(Units.inchesToMeters(120), 3.0);
    shooterAngleMap.put(Units.inchesToMeters(140), 1.9);
    shooterAngleMap.put(Units.inchesToMeters(160), 8.0);
    shooterAngleMap.put(Units.inchesToMeters(180), 11.0);

    // Units: seconds
    flightTimeMap.put(Units.inchesToMeters(40), 0.5);
    flightTimeMap.put(Units.inchesToMeters(150), 0.8);
  }

  public static AimingParameters calculate(
      Translation2d fieldRelativePose, Translation2d fieldRelativeVelocity) {
    Translation2d speakerPose =
        AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.getTranslation());
    double distanceToSpeaker = fieldRelativePose.getDistance(speakerPose);
    Translation2d effectiveAimingPose =
        fieldRelativePose.plus(fieldRelativeVelocity.times(flightTimeMap.get(distanceToSpeaker)));
    double effectiveDistanceToSpeaker = effectiveAimingPose.getDistance(speakerPose);

    Logger.recordOutput("ShotCalculator/effectiveDistanceToSpeaker", effectiveDistanceToSpeaker);
    Logger.recordOutput(
        "ShotCalculator/effectiveAimingPose", new Pose2d(effectiveAimingPose, new Rotation2d()));
    Logger.recordOutput(
        "ShotCalculator/robotAngle", speakerPose.minus(effectiveAimingPose).getAngle());
    return new AimingParameters(
        speakerPose.minus(effectiveAimingPose).getAngle(),
        shooterSpeedMap.get(effectiveDistanceToSpeaker),
        new Rotation2d(shooterAngleMap.get(effectiveDistanceToSpeaker)));
  }

  public static record AimingParameters(
      Rotation2d robotAngle, double shooterSpeed, Rotation2d shooterAngle) {}
}
