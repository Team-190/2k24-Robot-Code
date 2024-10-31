package frc.robot.subsystems.snapback.intake;

import frc.robot.Constants;

public class IntakeConstants {
  public static final int INTAKE_MOTOR_ID;
  public static final int SERIALIZER_MOTOR_ID;
  public static final int KICKER_MOTOR_ID;
  public static final int SENSOR_ID;
  public static final int PNEUMATIC_ID;
  public static final double INTAKE_GEAR_RATIO;
  public static final double SERIALIZER_GEAR_RATIO;
  public static final double KICKER_GEAR_RATIO;

  static {
    switch (Constants.ROBOT) {
      case SNAPBACK:
        INTAKE_MOTOR_ID = 0;
        SERIALIZER_MOTOR_ID = 1;
        KICKER_MOTOR_ID = 2;
        SENSOR_ID = 0;
        PNEUMATIC_ID = 5;
        INTAKE_GEAR_RATIO = 1.6;
        SERIALIZER_GEAR_RATIO = 2.0;
        KICKER_GEAR_RATIO = 2.0;
        break;
      default:
        INTAKE_MOTOR_ID = 0;
        SERIALIZER_MOTOR_ID = 1;
        KICKER_MOTOR_ID = 2;
        SENSOR_ID = 0;
        PNEUMATIC_ID = 0;
        INTAKE_GEAR_RATIO = 1.0;
        SERIALIZER_GEAR_RATIO = 1.0;
        KICKER_GEAR_RATIO = 1.0;

        break;
    }
  }
}
