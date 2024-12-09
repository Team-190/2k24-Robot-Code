package frc.robot.subsystems.whiplash.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class WhiplashArmConstants {

  public static final int ARM_CAN_ID;
  public static final double ARM_GEAR_RATIO;
  public static final double CURRENT_LIMIT;
  public static final double ARM_MOMENT_OF_INERTIA;
  public static final DCMotor ARM_MOTOR_CONFIG;
  public static final Rotation2d ARM_STOW_CONSTANT;
  public static final Rotation2d ARM_INTAKE_CONSTANT;
  public static final LoggedTunableNumber ARM_AMP_CONSTANT;
  public static final LoggedTunableNumber ARM_PRE_AMP_CONSTANT;
  public static final Rotation2d ARM_ABSOLUTE_ENCODER_OFFSET;
  public static final int ARM_ENCODER_ID;
  public static final LoggedTunableNumber GOAL_TOLERANCE;
  public static final double ARM_LENGTH_METERS;
  public static final double ARM_MIN_ANGLE;
  public static final double ARM_MAX_ANGLE;
  public static final int CANCODER_CAN_ID;
  public static final LoggedTunableNumber ARM_KP;
  public static final LoggedTunableNumber ARM_KD;
  public static final LoggedTunableNumber ARM_KS;
  public static final LoggedTunableNumber ARM_KG;
  public static final LoggedTunableNumber ARM_KV;
  public static final LoggedTunableNumber ARM_MAX_VELOCITY;
  public static final LoggedTunableNumber ARM_MAX_ACCELERATION;
  public static final LoggedTunableNumber ARM_SUBWOOFER_CONSTANT;
  public static final Rotation2d ARM_EJECT_ANGLE;
  public static final LoggedTunableNumber FEED_ANGLE;

  static {
    ARM_KP = new LoggedTunableNumber("Arm/KP");
    ARM_KD = new LoggedTunableNumber("Arm/KD");
    ARM_KS = new LoggedTunableNumber("Arm/KS");
    ARM_KG = new LoggedTunableNumber("Arm/KG");
    ARM_KV = new LoggedTunableNumber("Arm/KV");
    ARM_MAX_VELOCITY = new LoggedTunableNumber("Arm/MAX_VELOCITY");
    ARM_MAX_ACCELERATION = new LoggedTunableNumber("Arm/MAX_ACCELERATION");
    ARM_AMP_CONSTANT = new LoggedTunableNumber("Arm/Amp Angle");
    ARM_PRE_AMP_CONSTANT = new LoggedTunableNumber("Arm/Pre Amp Angle");
    ARM_SUBWOOFER_CONSTANT = new LoggedTunableNumber("Arm/Subwoofer Angle");
    GOAL_TOLERANCE = new LoggedTunableNumber("Arm/Goal Tolerance");
    FEED_ANGLE = new LoggedTunableNumber("Arm/Feed Angle");

    switch (Constants.ROBOT) {
      case WHIPLASH:
      default:
        ARM_CAN_ID = 13;
        CANCODER_CAN_ID = 24;

        // Get
        ARM_GEAR_RATIO = 60.666666666;

        CURRENT_LIMIT = 40.0;

        // Get
        ARM_MOMENT_OF_INERTIA = 0.004;

        ARM_MOTOR_CONFIG = DCMotor.getKrakenX60(1);

        ARM_STOW_CONSTANT = Rotation2d.fromDegrees(20.0);
        ARM_INTAKE_CONSTANT = ARM_STOW_CONSTANT;
        ARM_AMP_CONSTANT.initDefault(Units.degreesToRadians(110.0));
        ARM_PRE_AMP_CONSTANT.initDefault(Units.degreesToRadians(90.0));
        ARM_ABSOLUTE_ENCODER_OFFSET =
            Rotation2d.fromRadians(-0.6273981422452273).plus(Rotation2d.fromDegrees(18.746));
        ARM_ENCODER_ID = 24;
        GOAL_TOLERANCE.initDefault(1.0);
        // Get
        ARM_LENGTH_METERS = 0.381;
        ARM_MIN_ANGLE = Units.degreesToRadians(18.75);
        ARM_MAX_ANGLE = Units.degreesToRadians(-114.0);
        ARM_KP.initDefault(120.0);
        ARM_KD.initDefault(0.0);
        ARM_KS.initDefault(0.14578);
        ARM_KG.initDefault(0.14124);
        ARM_KV.initDefault(0.9053);
        ARM_MAX_VELOCITY.initDefault(120.0);
        ARM_MAX_ACCELERATION.initDefault(120.0);
        ARM_SUBWOOFER_CONSTANT.initDefault(Units.degreesToRadians(57.0));
        ARM_EJECT_ANGLE = Rotation2d.fromDegrees(45.0);
        FEED_ANGLE.initDefault(Units.degreesToRadians(40.0));
        break;
    }
  }
}
