package frc.robot.subsystems.snapback.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.Constants;

public class ShooterConstants {
  // Motor parameters
  public static final int LEFT_FLYWHEEL_MOTOR_CAN_ID;
  public static final int RIGHT_FLYWHEEL_MOTOR_CAN_ID;
  public static final int ACCELERATOR_MOTOR_CAN_ID;

  // Gear reduction parameters
  public static final double FLYWHEEL_GEAR_REDUCTION;
  public static final double ACCELERATOR_GEAR_REDUCTION;

  // Flywheel tolerance
  public static final double FLYWHEEL_TOLERANCE_RAD_PER_SEC;
  // Gains
  public static final Gains GAINS;

  // Simulation parameters
  public static final DCMotor FLYWHEEL_GEARBOX;
  public static final DCMotor ACCELERATOR_GEARBOX;

  static {
    switch (Constants.ROBOT) {
      case SNAPBACK:
        LEFT_FLYWHEEL_MOTOR_CAN_ID = 16;
        RIGHT_FLYWHEEL_MOTOR_CAN_ID = 10;
        ACCELERATOR_MOTOR_CAN_ID = 15;

        FLYWHEEL_GEAR_REDUCTION = 68.0 / 24.0;
        ACCELERATOR_GEAR_REDUCTION = 2.0;

        FLYWHEEL_TOLERANCE_RAD_PER_SEC = 0.1;
        GAINS = new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        break;
      default:
        LEFT_FLYWHEEL_MOTOR_CAN_ID = 0;
        RIGHT_FLYWHEEL_MOTOR_CAN_ID = 1;
        ACCELERATOR_MOTOR_CAN_ID = 2;

        FLYWHEEL_GEAR_REDUCTION = 1.0;
        ACCELERATOR_GEAR_REDUCTION = 1.0;

        FLYWHEEL_TOLERANCE_RAD_PER_SEC = 0.0;
        GAINS = new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }
    FLYWHEEL_GEARBOX = DCMotor.getKrakenX60Foc(1);
    ACCELERATOR_GEARBOX = DCMotor.getKrakenX60Foc(1);
  }

  public record Gains(double kp, double ki, double kd, double ks, double kv, double ka) {}
}
