package frc.robot.subsystems.snapback.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.Constants;
import frc.robot.util.LoggedTunableNumber;

public class ClimberConstants {
  // Motor parameters
  public static final int LEFT_CLIMBER_MOTOR_CAN_ID;
  public static final int RIGHT_CLIMBER_MOTOR_CAN_ID;

  // Gear reduction parameters
  public static final double CLIMBER_GEAR_REDUCTION;
  public static final double CLIMBER_PULLY_RADIUS_METERS;

  // Climber tolerance
  public static final LoggedTunableNumber CLIMBER_TOLERANCE_METERS;
  // Gains
  public static final Gains GAINS;

  // Flywheel constraints
  public static final LoggedTunableNumber CRUISE_VELOCITY;
  public static final LoggedTunableNumber MAX_ACCELERATION;

  // Simulation parameters
  public static final DCMotor CLIMBER_GEARBOX;

  static {
    CLIMBER_TOLERANCE_METERS = new LoggedTunableNumber("Climber/CLIMBER_TOLERANCE_METERS");

    CRUISE_VELOCITY = new LoggedTunableNumber("Climber/CRUISE_VELOCITY");
    MAX_ACCELERATION = new LoggedTunableNumber("Climber/MAX_ACCELERATION");

    switch (Constants.ROBOT) {
      case SNAPBACK:
        LEFT_CLIMBER_MOTOR_CAN_ID = 16;
        RIGHT_CLIMBER_MOTOR_CAN_ID = 10;

        CLIMBER_GEAR_REDUCTION = 68.0 / 24.0;
        CLIMBER_PULLY_RADIUS_METERS = 0.0;

        CLIMBER_TOLERANCE_METERS.initDefault(0.1);
        GAINS =
            new Gains(
                new LoggedTunableNumber("Climber/GAINS/kP"),
                new LoggedTunableNumber("Climber/GAINS/kI"),
                new LoggedTunableNumber("Climber/GAINS/kD"),
                0.0,
                0.0,
                0.0);

        CRUISE_VELOCITY.initDefault(0.0);
        MAX_ACCELERATION.initDefault(0.0);
        break;
      default:
        LEFT_CLIMBER_MOTOR_CAN_ID = 0;
        RIGHT_CLIMBER_MOTOR_CAN_ID = 1;

        CLIMBER_GEAR_REDUCTION = 1.0;
        CLIMBER_PULLY_RADIUS_METERS = 0.0;

        CLIMBER_TOLERANCE_METERS.initDefault(0.0);
        GAINS =
            new Gains(
                new LoggedTunableNumber("Climber/GAINS/kP"),
                new LoggedTunableNumber("Climber/GAINS/kI"),
                new LoggedTunableNumber("Climber/GAINS/kD"),
                0.0,
                0.0,
                0.0);

        CRUISE_VELOCITY.initDefault(0.0);
        MAX_ACCELERATION.initDefault(0.0);
    }
    CLIMBER_GEARBOX = DCMotor.getKrakenX60Foc(1);
  }

  public record Gains(
      LoggedTunableNumber kp,
      LoggedTunableNumber ki,
      LoggedTunableNumber kd,
      double ks,
      double kv,
      double ka) {}
}
