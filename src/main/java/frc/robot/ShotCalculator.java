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
    shooterSpeedMap.put(Units.inchesToMeters(60), 200.0);
    // shooterSpeedMap.put(Units.inchesToMeters(80), 10.0);
    // shooterSpeedMap.put(Units.inchesToMeters(100), 12.0);
    shooterSpeedMap.put(Units.inchesToMeters(120), 400.0);
    // shooterSpeedMap.put(Units.inchesToMeters(140), 19.0);
    // shooterSpeedMap.put(Units.inchesToMeters(160), 88.0);
    shooterSpeedMap.put(Units.inchesToMeters(180), 600.0);

    // Units: radians
    shooterAngleMap.put(Units.inchesToMeters(60), 3.0);
    // shooterAngleMap.put(Units.inchesToMeters(80), 2.5);
    // shooterAngleMap.put(Units.inchesToMeters(100), 2.0);
    shooterAngleMap.put(Units.inchesToMeters(120), 2.0);
    // shooterAngleMap.put(Units.inchesToMeters(140), 1.9);
    // shooterAngleMap.put(Units.inchesToMeters(160), 8.0);
    shooterAngleMap.put(Units.inchesToMeters(180), 1.0);

    // Units: seconds
    flightTimeMap.put(Units.inchesToMeters(40), 0.5);
    flightTimeMap.put(Units.inchesToMeters(150), 0.8);
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

  public static record AimingParameters(
      Rotation2d robotAngle, double radialVelocity, double shooterSpeed, Rotation2d shooterAngle) {}
}
