package frc.robot.subsystems.whiplash.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import lombok.RequiredArgsConstructor;

public class WhiplashArmConstants {

  public static final int MOTOR_CAN_ID;
  public static final int CANCODER_CAN_ID;
  public static final Rotation2d ABSOLUTE_ENCODER_OFFSET;

  public static final double GEAR_RATIO;
  public static final double CURRENT_LIMIT;
  public static final double MOMENT_OF_INERTIA;
  public static final DCMotor MOTOR_CONFIG;
  public static final double LENGTH_METERS;
  public static final double MIN_ANGLE;
  public static final double MAX_ANGLE;

  public static final Gains GAINS;
  public static final Constraints CONSTRAINTS;

  static {
    MOTOR_CAN_ID = 13;
    CANCODER_CAN_ID = 24;
    ABSOLUTE_ENCODER_OFFSET =
        Rotation2d.fromRadians(-0.6273981422452273).plus(Rotation2d.fromDegrees(18.746));

    GEAR_RATIO = 60.666666666;
    CURRENT_LIMIT = 40.0;
    MOMENT_OF_INERTIA = 0.004;
    MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);
    LENGTH_METERS = 0.381;
    MIN_ANGLE = Units.degreesToRadians(18.75);
    MAX_ANGLE = Units.degreesToRadians(-114.0);

    GAINS =
        new Gains(
            new LoggedTunableNumber("Arm/KS", 0.14578),
            new LoggedTunableNumber("Arm/KG", 0.14124),
            new LoggedTunableNumber("Arm/KV", 0.9053),
            new LoggedTunableNumber("Arm/KP", 120.0),
            new LoggedTunableNumber("Arm/KD", 0.0));
    CONSTRAINTS =
        new Constraints(
            new LoggedTunableNumber("Arm/Max Velocity", 120.0),
            new LoggedTunableNumber("Arm/Max Acceleration", 120.0),
            new LoggedTunableNumber("Arm/Goal Tolerance", Units.degreesToRadians(1.0)));
  }

  @RequiredArgsConstructor
  public enum WhiplashArmGoal {
    STOW(() -> Rotation2d.fromDegrees(20.0)),
    INTAKE(() -> Rotation2d.fromDegrees(20.0)),
    EJECT(() -> Rotation2d.fromDegrees(45.0)),
    PREAMP(() -> Rotation2d.fromDegrees(90.0)),
    AMP(() -> Rotation2d.fromDegrees(110.0)),
    SPEAKER(() -> RobotState.getControlData().speakerArmAngle()),
    FEED(() -> RobotState.getControlData().feedArmAngle()),
    SUBWOOFER(
        () ->
            shootForward()
                ? Rotation2d.fromDegrees(57.0)
                : AMP.getAngle().plus(Rotation2d.fromDegrees(3.5)));

    private final Supplier<Rotation2d> angle;

    public Rotation2d getAngle() {
      return angle.get();
    }

    public static boolean shootForward() {
      double angle = AllianceFlipUtil.apply(RobotState.getRobotPose().getRotation()).getDegrees();
      return (angle > -90 && angle < 90);
    }
  }

  public record Gains(
      LoggedTunableNumber ks,
      LoggedTunableNumber kg,
      LoggedTunableNumber kv,
      LoggedTunableNumber kp,
      LoggedTunableNumber kd) {}

  public record Constraints(
      LoggedTunableNumber maxVelocityRadiansPerSecond,
      LoggedTunableNumber maxAccelerationRadiansPerSecondSqaured,
      LoggedTunableNumber goalToleranceRadians) {}
}
