package frc.robot.subsystems.snapback.hood;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import frc.robot.util.LoggedTunableNumber;

public class HoodConstants {

  public static final Gains GAINS;
  public static final int HOOD_MOTOR_CAN_ID;
  public static final double GEAR_REDUCTION;
  public static final DCMotor HOOD_GEARBOX;
  public static final LoggedTunableNumber CRUISING_VELOCITY =
      new LoggedTunableNumber("Hood/Max Velocity");
  public static final LoggedTunableNumber MAX_ACCELERATION =
      new LoggedTunableNumber("Hood/Max Acceleration");
  public static final LoggedTunableNumber STOWED_POSITION =
      new LoggedTunableNumber("Hood/Stowed Position");
  public static final LoggedTunableNumber AMP_POSITION =
      new LoggedTunableNumber("Hood/Amp Position");
  public static final LoggedTunableNumber SOURCE_SIDE_FEED_POSITION =
      new LoggedTunableNumber("Hood/Source Side Feed Position");
  public static final LoggedTunableNumber AMP_SIDE_FEED_POSITION =
      new LoggedTunableNumber("Hood/Amp Side Feed Position");
  public static final LoggedTunableNumber MIN_POSITION =
      new LoggedTunableNumber("Hood/Minimum Angle");
  public static final LoggedTunableNumber MAX_POSITION =
      new LoggedTunableNumber("Hood/Maximum Angle");
  public static final LoggedTunableNumber GOAL_TOLERANCE =
      new LoggedTunableNumber("Hood/Goal Tolerance");

  static {
    GOAL_TOLERANCE.initDefault(0.017);
    switch (Constants.ROBOT) {
      case SNAPBACK:
        HOOD_MOTOR_CAN_ID = 7;
        GEAR_REDUCTION = 85.0;
        CRUISING_VELOCITY.initDefault(50.0);
        MAX_ACCELERATION.initDefault(40.0);
        STOWED_POSITION.initDefault(0.0);
        AMP_POSITION.initDefault(0.3);
        SOURCE_SIDE_FEED_POSITION.initDefault(0.3);
        AMP_SIDE_FEED_POSITION.initDefault(0.5);
        MIN_POSITION.initDefault(0.0);
        MAX_POSITION.initDefault(0.75);
        GAINS = new Gains(25.0, 0.0, 0.01, 0.0, 0.0, 0.0);
        break;
      default:
        HOOD_MOTOR_CAN_ID = 0;
        GEAR_REDUCTION = 1;
        CRUISING_VELOCITY.initDefault(1000.0);
        MAX_ACCELERATION.initDefault(1000.0);
        STOWED_POSITION.initDefault(Units.degreesToRadians(38.0));
        AMP_POSITION.initDefault(Units.degreesToRadians(15.0));
        MIN_POSITION.initDefault(0.0);
        MAX_POSITION.initDefault(0.75);
        GAINS = new Gains(90.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        break;
    }
    HOOD_GEARBOX = DCMotor.getKrakenX60Foc(1);
  }

  public record Gains(double kp, double ki, double kd, double ks, double kv, double ka) {}
}
