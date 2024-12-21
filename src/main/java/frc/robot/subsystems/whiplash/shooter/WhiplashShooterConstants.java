package frc.robot.subsystems.whiplash.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;

/** The ShooterConstants class */
public class WhiplashShooterConstants {

  public static final int TOP_CAN_ID;
  public static final int BOTTOM_CAN_ID;

  public static final double CURRENT_LIMIT;
  public static final double TOP_MOMENT_OF_INERTIA;
  public static final double BOTTOM_MOMENT_OF_INERTIA;

  public static final DCMotor TOP_MOTOR_CONFIG;
  public static final DCMotor BOTTOM_MOTOR_CONFIG;

  public static final Gains GAINS;
  public static final Constraints CONSTRAINTS;

  static {
    TOP_CAN_ID = 14;
    BOTTOM_CAN_ID = 15;

    CURRENT_LIMIT = 40.0;
    TOP_MOMENT_OF_INERTIA = 0.004;
    BOTTOM_MOMENT_OF_INERTIA = 0.004;

    TOP_MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);
    BOTTOM_MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);

    GAINS =
        new Gains(
            new LoggedTunableNumber("Shooter/kP", 0.325),
            new LoggedTunableNumber("Shooter/kD", 0.0),
            new LoggedTunableNumber("Shooter/kS", 0.090597),
            new LoggedTunableNumber("Shooter/kV", 12.0 / 103.4508),
            new LoggedTunableNumber("Shooter/kA", 0.0014107));
    CONSTRAINTS =
        new Constraints(
            new LoggedTunableNumber("Shooter/Max Acceleration", 100.0),
            new LoggedTunableNumber("Shooter/Goal Tolerance", 15.0));
  }

  @RequiredArgsConstructor
  public enum WhiplashShooterGoal {
    IDLE(() -> 0.0, () -> 0.0),
    SPEAKER(
        () -> RobotState.getControlData().speakerShotSpeed().f1Speed(),
        () -> RobotState.getControlData().speakerShotSpeed().f2Speed()),
    FEED(
        () -> RobotState.getControlData().feedShotSpeed().f1Speed(),
        () -> RobotState.getControlData().feedShotSpeed().f2Speed()),
    AMP(() -> 60.0, () -> 40.0);

    private final DoubleSupplier topGoal;
    private final DoubleSupplier bottomGoal;

    public double getTopGoal() {
      return topGoal.getAsDouble();
    }

    public double getBottomGoal() {
      return bottomGoal.getAsDouble();
    }
  }

  public record Gains(
      LoggedTunableNumber kp,
      LoggedTunableNumber kd,
      LoggedTunableNumber ks,
      LoggedTunableNumber kv,
      LoggedTunableNumber ka) {}

  public record Constraints(
      LoggedTunableNumber maxAccelerationRadiansPerSecondSquared,
      LoggedTunableNumber goalToleranceRadiansPerSecond) {}
}
