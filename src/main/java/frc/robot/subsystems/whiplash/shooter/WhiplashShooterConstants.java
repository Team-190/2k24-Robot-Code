package frc.robot.subsystems.whiplash.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

/** The ShooterConstants class */
public class WhiplashShooterConstants {

  public static final int TOP_CAN_ID;
  public static final int BOTTOM_CAN_ID;
  public static final double CURRENT_LIMIT;
  public static final double TOP_MOMENT_OF_INERTIA;
  public static final double BOTTOM_MOMENT_OF_INERTIA;
  public static final DCMotor TOP_MOTOR_CONFIG;
  public static final DCMotor BOTTOM_MOTOR_CONFIG;
  public static final LoggedTunableNumber KP;
  public static final LoggedTunableNumber KD;
  public static final LoggedTunableNumber KS;
  public static final LoggedTunableNumber KV;
  public static final LoggedTunableNumber KA;
  public static final LoggedTunableNumber MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED;
  public static final LoggedTunableNumber TOP_AMP_SPEED;
  public static final LoggedTunableNumber BOTTOM_AMP_SPEED;
  public static final LoggedTunableNumber SPEAKER_SPEED;
  public static final LoggedTunableNumber FEED_SPEED;
  public static final double SPEED_TOLERANCE_RADIANS_PER_SECOND;

  static {
    KP = new LoggedTunableNumber("Shooter/kP");
    KD = new LoggedTunableNumber("Shooter/kD");
    KS = new LoggedTunableNumber("Shooter/kS");
    KV = new LoggedTunableNumber("Shooter/kV");
    KA = new LoggedTunableNumber("Shooter/kA");
    MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED =
        new LoggedTunableNumber("Shooter/Max Acceleration");
    TOP_AMP_SPEED = new LoggedTunableNumber("Shooter/Top Amp Speed");
    BOTTOM_AMP_SPEED = new LoggedTunableNumber("Shooter/Bottom Amp Speed");
    SPEAKER_SPEED = new LoggedTunableNumber("Shooter/Speaker Speed");
    FEED_SPEED = new LoggedTunableNumber("Shooter/Feed Speed");

    switch (Constants.ROBOT) {
      default:
        TOP_CAN_ID = 14;
        BOTTOM_CAN_ID = 15;
        CURRENT_LIMIT = 40.0;
        TOP_MOMENT_OF_INERTIA = 0.004;
        BOTTOM_MOMENT_OF_INERTIA = 0.004;
        TOP_MOTOR_CONFIG = DCMotor.getKrakenX60(1);
        BOTTOM_MOTOR_CONFIG = DCMotor.getKrakenX60(1);
        KP.initDefault(0.325);
        KD.initDefault(0.0);
        KS.initDefault(0.090597);
        KV.initDefault(12.0 / 103.4508);
        KA.initDefault(0.0014107);
        SPEED_TOLERANCE_RADIANS_PER_SECOND = 15.0;
        TOP_AMP_SPEED.initDefault(60.0);
        BOTTOM_AMP_SPEED.initDefault(40.0);
        SPEAKER_SPEED.initDefault(600.0);
        FEED_SPEED.initDefault(300);
        break;
    }
  }
}
