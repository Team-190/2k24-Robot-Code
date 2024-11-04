package frc.robot.subsystems.snapback.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.Constants;

public class IntakeConstants {
  public static final int INTAKE_MOTOR_CAN_ID;
  public static final int SERIALIZER_MOTOR_CAN_ID;
  public static final int KICKER_MOTOR_CAN_ID;

  public static final double INTAKE_GEAR_REDUCTION;
  public static final double SERIALIZER_GEAR_REDUCTION;
  public static final double KICKER_GEAR_REDUCTION;

  public static final int SENSOR_CHANNEL;

  public static final int PNEUMATIC_FORWARD_CHANNEL;
  public static final int PNEUMATIC_REVERSE_CHANNEL;

  public static final DCMotor MOTOR_CONFIG;

  static {
    switch (Constants.ROBOT) {
      case SNAPBACK:
        INTAKE_MOTOR_CAN_ID = 1;
        SERIALIZER_MOTOR_CAN_ID = 2;
        KICKER_MOTOR_CAN_ID = 3;

        INTAKE_GEAR_REDUCTION = 1.6;
        SERIALIZER_GEAR_REDUCTION = 2.0;
        KICKER_GEAR_REDUCTION = 2.0;

        SENSOR_CHANNEL = 0;

        PNEUMATIC_FORWARD_CHANNEL = 5;
        PNEUMATIC_REVERSE_CHANNEL = 6;

        MOTOR_CONFIG = DCMotor.getKrakenX60(1);

        break;
      default:
        INTAKE_MOTOR_CAN_ID = 0;
        SERIALIZER_MOTOR_CAN_ID = 0;
        KICKER_MOTOR_CAN_ID = 0;

        INTAKE_GEAR_REDUCTION = 1.6;
        SERIALIZER_GEAR_REDUCTION = 2.0;
        KICKER_GEAR_REDUCTION = 2.0;

        SENSOR_CHANNEL = 0;

        PNEUMATIC_FORWARD_CHANNEL = 5;
        PNEUMATIC_REVERSE_CHANNEL = 6;

        MOTOR_CONFIG = DCMotor.getKrakenX60(1);
    }
  }
}
