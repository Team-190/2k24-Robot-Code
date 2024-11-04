package frc.robot.subsystems.snapback.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.Constants;

public class IntakeConstants {
  // Motor parameters
  public static final int INTAKE_MOTOR_CAN_ID;
  public static final int SERIALIZER_MOTOR_CAN_ID;
  public static final int KICKER_MOTOR_CAN_ID;

  // Gear reduction parameters
  public static final double INTAKE_GEAR_REDUCTION;
  public static final double SERIALIZER_GEAR_REDUCTION;
  public static final double KICKER_GEAR_REDUCTION;

  // Beam Break sensor parameters
  public static final int SENSOR_CHANNEL;

  // Pneumatic parameters
  public static final int PNEUMATIC_FORWARD_CHANNEL;
  public static final int PNEUMATIC_REVERSE_CHANNEL;

  // Simulations parameters
  public static final DCMotor INTAKE_GEARBOX;
  public static final DCMotor SERIALIZER_GEARBOX;
  public static final DCMotor KICKER_GEARBOX;

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
    }

    INTAKE_GEARBOX = DCMotor.getKrakenX60(1);
    SERIALIZER_GEARBOX = DCMotor.getKrakenX60(1);
    KICKER_GEARBOX = DCMotor.getKrakenX60(1);
  }
}
