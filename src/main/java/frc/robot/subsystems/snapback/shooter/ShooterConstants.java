package frc.robot.subsystems.snapback.shooter;

import frc.robot.Constants;

public class ShooterConstants {
  public static final int LEFT_FLYWHEEL_MOTOR_CAN_ID;
  public static final int RIGHT_FLYWHEEL_MOTOR_CAN_ID;
  public static final int ACCELERATOR_MOTOR_CAN_ID;
  public static final double FLYWHEEL_GEAR_RATIO;
  public static final double ACCELERATOR_GEAR_RATIO;

  static {
    switch (Constants.ROBOT) {
      case SNAPBACK:
        LEFT_FLYWHEEL_MOTOR_CAN_ID = 16;
        RIGHT_FLYWHEEL_MOTOR_CAN_ID = 10;
        ACCELERATOR_MOTOR_CAN_ID = 15;
        FLYWHEEL_GEAR_RATIO = 68.0 / 24.0;
        ACCELERATOR_GEAR_RATIO = 2.0;
        break;
      default:
        LEFT_FLYWHEEL_MOTOR_CAN_ID = 0;
        RIGHT_FLYWHEEL_MOTOR_CAN_ID = 1;
        ACCELERATOR_MOTOR_CAN_ID = 2;
        FLYWHEEL_GEAR_RATIO = 1.0;
        ACCELERATOR_GEAR_RATIO = 1.0;
    }
  }

  public record Gains(double kp, double ki, double kd, double ks, double kv, double ka) {}
}
