package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
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
    // Units: radians/second          possibly 775.0
    shooterSpeedMap.put(2.16, 800.0);
    shooterSpeedMap.put(2.45, 800.0);
    shooterSpeedMap.put(2.69, 800.0);
    shooterAngleMap.put(2.84, 800.0);
    shooterSpeedMap.put(2.98, 800.0);
    shooterSpeedMap.put(3.19, 800.0);
    shooterSpeedMap.put(3.52, 800.0);
    shooterSpeedMap.put(3.85, 900.0);
    shooterSpeedMap.put(4.29, 900.0);

    // Units: radians
    shooterAngleMap.put(2.16, 0.05);
    shooterAngleMap.put(2.45, 0.05);
    shooterAngleMap.put(2.69, 0.35);
    shooterAngleMap.put(2.84, 0.41 + Units.degreesToRadians(-3)); // -3 degrees
    // shooterAngleMap.put(2.98, 0.415);
    shooterAngleMap.put(3.19, 0.42 + Units.degreesToRadians(-1.5)); // -1.5
    shooterAngleMap.put(3.52, 0.43 + Units.degreesToRadians(-1)); // -1
    shooterAngleMap.put(3.85, 0.465 + Units.degreesToRadians(-1)); // -1
    shooterAngleMap.put(4.29, 0.48 + Units.degreesToRadians(-0.5)); // -0.5

    // Units: seconds
    flightTimeMap.put(2.50, (4.42 - 4.24));
    flightTimeMap.put(2.75, (2.56 - 2.33));
    flightTimeMap.put(3.00, (3.43 - 3.18));
    flightTimeMap.put(3.25, (3.20 - 2.94));
    flightTimeMap.put(3.50, (2.64 - 2.42));
    flightTimeMap.put(4.0, (2.60 - 2.32));

    SHOOTER_SPEED_TOLERANCE.initDefault(40.0);
    HOOD_ANGLE_TOLERANCE.initDefault(Units.degreesToRadians(2));
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

  public static boolean shooterReady(Hood hood, Shooter shooter) {
    return shooter.atGoal() && hood.atGoal();
  }

  public static record AimingParameters(
      Rotation2d robotAngle, double radialVelocity, double shooterSpeed, Rotation2d shooterAngle) {}
}
