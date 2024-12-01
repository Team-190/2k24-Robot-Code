package frc.robot.subsystems.snapback.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.Constants;
import frc.robot.util.LoggedTunableNumber;

public class ClimberConstants {
  // Motor CAN IDs
  public static final int LEFT_CLIMBER_MOTOR_CAN_ID;
  public static final int RIGHT_CLIMBER_MOTOR_CAN_ID;

  // Gear reductions
  public static final double CLIMBER_GEAR_REDUCTION;
  public static final double CLIMBER_PULLY_RADIUS_METERS;

  // Position tolerance
  public static final LoggedTunableNumber CLIMBER_TOLERANCE_METERS;

  // Climber Position Constants
  public static final double CLIMBER_MAX_HEIGHT_METERS;
  public static final double CLIMBER_MIN_HEIGHT_METERS;
  public static final LoggedTunableNumber CLIMBER_STOWED_HEIGHT_METERS;

  // Physical parameters
  public static final DCMotor CLIMBER_GEARBOX;

  static {
    CLIMBER_TOLERANCE_METERS = new LoggedTunableNumber("Climber/CLIMBER_TOLERANCE_METERS");
    CLIMBER_STOWED_HEIGHT_METERS = new LoggedTunableNumber("Climber/CLIMBER_STOWED_HEIGHT_METERS");

    switch (Constants.ROBOT) {
      case SNAPBACK:
      default:
        LEFT_CLIMBER_MOTOR_CAN_ID = 16;
        RIGHT_CLIMBER_MOTOR_CAN_ID = 10;

        CLIMBER_GEAR_REDUCTION = 68.0 / 24.0;
        CLIMBER_PULLY_RADIUS_METERS = 0.0;

        CLIMBER_TOLERANCE_METERS.initDefault(0.001);

        CLIMBER_MAX_HEIGHT_METERS = 0.0;
        CLIMBER_MIN_HEIGHT_METERS = 0.0;
        CLIMBER_STOWED_HEIGHT_METERS.initDefault(0.0);
        break;
    }
    CLIMBER_GEARBOX = DCMotor.getKrakenX60Foc(1);
  }
}
