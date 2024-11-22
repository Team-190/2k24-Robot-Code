package frc.robot.subsystems.snapback.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.Constants;

public class ClimberConstants {
  // Motor parameters
  public static final int LEFT_CLIMBER_MOTOR_CAN_ID;
  public static final int RIGHT_CLIMBER_MOTOR_CAN_ID;

  // Gear reduction parameters
  public static final double CLIMBER_GEAR_REDUCTION;
  public static final double CLIMBER_PULLY_RADIUS_METERS;

  // Climber tolerance
  public static final double CLIMBER_TOLERANCE_METERS;
  // Gains
  public static final Gains GAINS;

  // Flywheel constraints
  public static final double CRUISE_VELOCITY;
  public static final double MAX_ACCELERATION;

  // Simulation parameters
  public static final DCMotor CLIMBER_GEARBOX;

  static {
    switch (Constants.ROBOT) {
      case SNAPBACK:
        LEFT_CLIMBER_MOTOR_CAN_ID = 16;
        RIGHT_CLIMBER_MOTOR_CAN_ID = 10;

        CLIMBER_GEAR_REDUCTION = 68.0 / 24.0;
        CLIMBER_PULLY_RADIUS_METERS = 0.0;

        CLIMBER_TOLERANCE_METERS = 0.1;
        GAINS = new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        CRUISE_VELOCITY = 0.0;
        MAX_ACCELERATION = 0.0;
        break;
      default:
        LEFT_CLIMBER_MOTOR_CAN_ID = 0;
        RIGHT_CLIMBER_MOTOR_CAN_ID = 1;

        CLIMBER_GEAR_REDUCTION = 1.0;
        CLIMBER_PULLY_RADIUS_METERS = 0.0;

        CLIMBER_TOLERANCE_METERS = 0.0;
        GAINS = new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        CRUISE_VELOCITY = 0.0;
        MAX_ACCELERATION = 0.0;
    }
    CLIMBER_GEARBOX = DCMotor.getKrakenX60Foc(1);
  }

  public record Gains(double kp, double ki, double kd, double ks, double kv, double ka) {}
}
