package frc.robot.subsystems.snapback.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;

public class SnapbackShooterConstants {
  public static final int LEFT_CAN_ID;
  public static final int RIGHT_CAN_ID;
  public static final int ACCELERATOR_CAN_ID;

  public static final double CURRENT_LIMIT;

  public static final double LEFT_GEAR_RATIO;
  public static final double RIGHT_GEAR_RATIO;
  public static final double ACCELERATOR_GEAR_RATIO;

  public static final double LEFT_MOMENT_OF_INERTIA;
  public static final double RIGHT_MOMENT_OF_INERTIA;
  public static final double ACCELERATOR_MOMENT_OF_INERTIA;

  public static final DCMotor LEFT_MOTOR_CONFIG;
  public static final DCMotor RIGHT_MOTOR_CONFIG;
  public static final DCMotor ACCELERATOR_MOTOR_CONFIG;

  public static final Gains GAINS;
  public static final Constraints CONSTRAINTS;

  static {
    LEFT_CAN_ID = 14;
    RIGHT_CAN_ID = 15;
    ACCELERATOR_CAN_ID = 16;

    CURRENT_LIMIT = 40.0;

    LEFT_GEAR_RATIO = 68.0 / 24.0;
    RIGHT_GEAR_RATIO = 68.0 / 24.0;
    ACCELERATOR_GEAR_RATIO = 2.0;

    LEFT_MOMENT_OF_INERTIA = 0.004;
    RIGHT_MOMENT_OF_INERTIA = 0.004;
    ACCELERATOR_MOMENT_OF_INERTIA = 0.004;

    LEFT_MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);
    RIGHT_MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);
    ACCELERATOR_MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);

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
  public enum SnapbackShooterGoal {
    IDLE(() -> 0.0, () -> 0.0),
    SPEAKER(
        () -> RobotState.getControlData().speakerShotSpeed().f1Speed(),
        () -> RobotState.getControlData().speakerShotSpeed().f2Speed()),
    FEED(
        () -> RobotState.getControlData().feedShotSpeed().f1Speed(),
        () -> RobotState.getControlData().feedShotSpeed().f2Speed());

    private final DoubleSupplier leftGoal;
    private final DoubleSupplier rightGoal;

    public double getLeftGoal() {
      return leftGoal.getAsDouble();
    }

    public double getRightGoal() {
      return rightGoal.getAsDouble();
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
