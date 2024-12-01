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
  public static final LoggedTunableNumber STOWED_POSITION;
  public static final LoggedTunableNumber AMP_POSITION;
  public static final LoggedTunableNumber SOURCE_SIDE_FEED_POSITION;
  public static final LoggedTunableNumber AMP_SIDE_FEED_POSITION;
  public static final LoggedTunableNumber MIN_POSITION;
  public static final LoggedTunableNumber MAX_POSITION;
  public static final LoggedTunableNumber GOAL_TOLERANCE;
  public static final LoggedTunableNumber MAX_ACCELERATION;
  public static final LoggedTunableNumber MAX_VELOCITY;
  public static final double SUPPLY_CURRENT_LIMIT_AMPS;

  static {
    STOWED_POSITION = new LoggedTunableNumber("Hood/Stowed Position");
    AMP_POSITION = new LoggedTunableNumber("Hood/Amp Position");
    SOURCE_SIDE_FEED_POSITION = new LoggedTunableNumber("Hood/Source Side Feed Position");
    AMP_SIDE_FEED_POSITION = new LoggedTunableNumber("Hood/Amp Side Feed Position");
    MIN_POSITION = new LoggedTunableNumber("Hood/Minimum Angle");
    MAX_POSITION = new LoggedTunableNumber("Hood/Maximum Angle");
    GOAL_TOLERANCE = new LoggedTunableNumber("Hood/Goal Tolerance");
    MAX_ACCELERATION = new LoggedTunableNumber("Hood/Maximum Acceleration");
    MAX_VELOCITY = new LoggedTunableNumber("Hood/Maximum Velocity");
    GOAL_TOLERANCE.initDefault(0.017);
    switch (Constants.ROBOT) {
      case SNAPBACK:
        HOOD_MOTOR_CAN_ID = 7;
        GEAR_REDUCTION = 85.0;
        STOWED_POSITION.initDefault(0.0);
        AMP_POSITION.initDefault(0.3);
        SOURCE_SIDE_FEED_POSITION.initDefault(0.3);
        AMP_SIDE_FEED_POSITION.initDefault(0.5);
        MIN_POSITION.initDefault(0.0);
        MAX_POSITION.initDefault(0.75);
        MAX_ACCELERATION.initDefault(40);
        MAX_VELOCITY.initDefault(50);
        GAINS = new Gains(25.0, 0.0, 0.01, 0.0, 0, 0);
        SUPPLY_CURRENT_LIMIT_AMPS = 60.0;
        break;
      default:
        HOOD_MOTOR_CAN_ID = 0;
        GEAR_REDUCTION = 1;
        STOWED_POSITION.initDefault(Units.degreesToRadians(38.0));
        AMP_POSITION.initDefault(Units.degreesToRadians(15.0));
        MIN_POSITION.initDefault(0.0);
        MAX_POSITION.initDefault(0.75);
        MAX_ACCELERATION.initDefault(1000);
        MAX_VELOCITY.initDefault(1000);
        GAINS = new Gains(90.0, 0.0, 0.01, 0.0, 0, 0);
        SUPPLY_CURRENT_LIMIT_AMPS = 60.0;
        break;
    }
    HOOD_GEARBOX = DCMotor.getKrakenX60Foc(1);
  }

  public record Gains(double kp, double ki, double kd, double ks, double kv, double ka) {}
}
