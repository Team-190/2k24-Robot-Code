package frc.robot.subsystems.whiplash.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.RobotState;
import frc.robot.constants.Constants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;

public class ShooterConstants {
  // Motor parameters
  public static final int BOTTOM_FLYWHEEL_MOTOR_CAN_ID;
  public static final int TOP_FLYWHEEL_MOTOR_CAN_ID;

  // Gear reduction parameters
  public static final double FLYWHEEL_GEAR_REDUCTION;

  // Current Limits
  public static final double FLYWHEEL_CURRENT_LIMIT;

  // Flywheel tolerance
  public static final double FLYWHEEL_TOLERANCE_RAD_PER_SEC;
  // Gains
  public static final Gains GAINS;
  // Flywheel constraints
  public static final LoggedTunableNumber CRUISE_VELOCITY;
  public static final LoggedTunableNumber MAX_ACCELERATION;
  // Simulation parameters
  public static final DCMotor FLYWHEEL_GEARBOX;

  static {
    CRUISE_VELOCITY = new LoggedTunableNumber("Shooter/Constraints/Cruise Velocity");
    MAX_ACCELERATION = new LoggedTunableNumber("Shooter/Constraints/Max Acceleration");
    switch (Constants.ROBOT) {
      case SNAPBACK:
        BOTTOM_FLYWHEEL_MOTOR_CAN_ID = 15;
        TOP_FLYWHEEL_MOTOR_CAN_ID = 14;

        FLYWHEEL_GEAR_REDUCTION = 68.0 / 24.0;

        FLYWHEEL_CURRENT_LIMIT = 40.0;

        FLYWHEEL_TOLERANCE_RAD_PER_SEC = 5.0;

        GAINS =
            new Gains(
                new LoggedTunableNumber("Shooter/GAINS/kP", 0.0),
                new LoggedTunableNumber("Shooter/GAINS/kI", 0.0),
                new LoggedTunableNumber("Shooter/GAINS/kD", 0.0),
                0.0,
                0.0,
                0.0);

        CRUISE_VELOCITY.initDefault(0.0);
        MAX_ACCELERATION.initDefault(0.0);
        GAINS.kp().initDefault(0.0);
        GAINS.ki().initDefault(0.0);
        GAINS.kd().initDefault(0.0);
        break;
      default:
        BOTTOM_FLYWHEEL_MOTOR_CAN_ID = 0;
        TOP_FLYWHEEL_MOTOR_CAN_ID = 1;

        FLYWHEEL_GEAR_REDUCTION = 1.0;

        FLYWHEEL_CURRENT_LIMIT = 0.0;

        FLYWHEEL_TOLERANCE_RAD_PER_SEC = 0.0;

        GAINS =
            new Gains(
                new LoggedTunableNumber("Shooter/GAINS/kP", 0.0),
                new LoggedTunableNumber("Shooter/GAINS/kI", 0.0),
                new LoggedTunableNumber("Shooter/GAINS/kD", 0.0),
                0.0,
                0.0,
                0.0);

        CRUISE_VELOCITY.initDefault(0.0);
        MAX_ACCELERATION.initDefault(0.0);
        GAINS.kp().initDefault(0.0);
        GAINS.ki().initDefault(0.0);
        GAINS.kd().initDefault(0.0);
    }
    FLYWHEEL_GEARBOX = DCMotor.getKrakenX60Foc(1);
  }

  public record Gains(
      LoggedTunableNumber kp,
      LoggedTunableNumber ki,
      LoggedTunableNumber kd,
      double ks,
      double kv,
      double ka) {}

  @RequiredArgsConstructor
  public enum Goal {
    IDLE(() -> 0.0, () -> 0.0),
    SPEAKER(
        () -> RobotState.getControlData().speakerShooterSpeed().f1Speed(),
        () -> RobotState.getControlData().speakerShooterSpeed().f2Speed()),
    FEED(
        () -> RobotState.getControlData().ampFeedShooterSpeed().f1Speed(),
        () -> RobotState.getControlData().ampFeedShooterSpeed().f2Speed());

    private final DoubleSupplier leftGoal;
    private final DoubleSupplier rightGoal;

    public double getBottomGoal() {
      return leftGoal.getAsDouble();
    }

    public double getTopGoal() {
      return rightGoal.getAsDouble();
    }
  }
}
