package frc.robot.subsystems.shared.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import frc.robot.constants.SnapbackTunerConstants;
import frc.robot.constants.WhiplashTunerConstants;

public class DriveConstants {
  // CAN
  public static final String CAN_BUS;
  public static final int GYRO_CAN_ID;
  public static final double ODOMETRY_FREQUENCY;

  // Kinematics
  public static final double TRACK_WIDTH_X;
  public static final double TRACK_WIDTH_Y;
  public static final double WHEEL_RADIUS;
  public static final double MAX_LINEAR_VELOCITY;
  public static final double MAX_ANGULAR_VELOCITY;
  public static final double DRIVE_BASE_RADIUS;
  public static final SwerveDriveKinematics KINEMATICS;

  // Modules
  public static final SwerveModuleConstants FRONT_LEFT;
  public static final SwerveModuleConstants FRONT_RIGHT;
  public static final SwerveModuleConstants BACK_LEFT;
  public static final SwerveModuleConstants BACK_RIGHT;

  // Drive
  public static final double DRIVER_DEADBAND;

  // Module Gains
  public static final double DRIVE_KP;
  public static final double DRIVE_KD;
  public static final double TURN_KP;
  public static final double TURN_KD;

  public static final double DRIVE_KS;
  public static final double DRIVE_KV;

  // Autonomous Gains
  public static final double AUTO_X_KP;
  public static final double AUTO_X_KD;

  public static final double AUTO_Y_KP;
  public static final double AUTO_Y_KD;

  public static final double AUTO_THETA_KP;
  public static final double AUTO_THETA_KD;

  // Simulation parameters
  public static final DCMotor DRIVE_GEARBOX;
  public static final DCMotor TURN_GEARBOX;

  static {
    switch (Constants.ROBOT) {
      case WHIPLASH:
        CAN_BUS = WhiplashTunerConstants.DrivetrainConstants.CANBusName;
        GYRO_CAN_ID = WhiplashTunerConstants.DrivetrainConstants.Pigeon2Id;
        ODOMETRY_FREQUENCY = new CANBus(CAN_BUS).isNetworkFD() ? 250.0 : 100.0;

        TRACK_WIDTH_X =
            Math.abs(WhiplashTunerConstants.FrontLeft.LocationX)
                + Math.abs(WhiplashTunerConstants.FrontRight.LocationX);
        TRACK_WIDTH_Y =
            Math.abs(WhiplashTunerConstants.FrontLeft.LocationY)
                + Math.abs(WhiplashTunerConstants.BackLeft.LocationY);
        WHEEL_RADIUS = WhiplashTunerConstants.kWheelRadius.in(Meters);
        MAX_LINEAR_VELOCITY = WhiplashTunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        DRIVE_BASE_RADIUS =
            Math.max(
                Math.max(
                    Math.hypot(
                        WhiplashTunerConstants.FrontLeft.LocationX,
                        WhiplashTunerConstants.FrontRight.LocationY),
                    Math.hypot(
                        WhiplashTunerConstants.FrontRight.LocationX,
                        WhiplashTunerConstants.FrontRight.LocationY)),
                Math.max(
                    Math.hypot(
                        WhiplashTunerConstants.BackLeft.LocationX,
                        WhiplashTunerConstants.BackLeft.LocationY),
                    Math.hypot(
                        WhiplashTunerConstants.BackRight.LocationX,
                        WhiplashTunerConstants.BackRight.LocationY)));
        MAX_ANGULAR_VELOCITY = (MAX_LINEAR_VELOCITY / DRIVE_BASE_RADIUS);
        KINEMATICS =
            new SwerveDriveKinematics(
                new Translation2d[] {
                  new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
                  new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
                  new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
                  new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
                });

        FRONT_LEFT = WhiplashTunerConstants.FrontLeft;
        FRONT_RIGHT = WhiplashTunerConstants.FrontRight;
        BACK_LEFT = WhiplashTunerConstants.BackLeft;
        BACK_RIGHT = WhiplashTunerConstants.BackRight;

        DRIVER_DEADBAND = 0.025;

        DRIVE_KP = WhiplashTunerConstants.driveGains.kP;
        DRIVE_KD = WhiplashTunerConstants.driveGains.kD;
        TURN_KP = WhiplashTunerConstants.steerGains.kP;
        TURN_KD = WhiplashTunerConstants.steerGains.kD;

        DRIVE_KS = WhiplashTunerConstants.driveGains.kS;
        DRIVE_KV = WhiplashTunerConstants.driveGains.kV;

        AUTO_X_KP = 4.0;
        AUTO_X_KD = 0.0;

        AUTO_Y_KP = 4.0;
        AUTO_Y_KD = 0.0;

        AUTO_THETA_KP = 5.0;
        AUTO_THETA_KD = 0.05;
        break;
      case SNAPBACK:
        CAN_BUS = SnapbackTunerConstants.DrivetrainConstants.CANBusName;
        GYRO_CAN_ID = SnapbackTunerConstants.DrivetrainConstants.Pigeon2Id;
        ODOMETRY_FREQUENCY = new CANBus(CAN_BUS).isNetworkFD() ? 250.0 : 100.0;

        TRACK_WIDTH_X =
            Math.abs(SnapbackTunerConstants.FrontLeft.LocationX)
                + Math.abs(SnapbackTunerConstants.FrontRight.LocationX);
        TRACK_WIDTH_Y =
            Math.abs(SnapbackTunerConstants.FrontLeft.LocationY)
                + Math.abs(SnapbackTunerConstants.BackLeft.LocationY);
        WHEEL_RADIUS = SnapbackTunerConstants.kWheelRadius.in(Meters);
        MAX_LINEAR_VELOCITY = SnapbackTunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        DRIVE_BASE_RADIUS =
            Math.max(
                Math.max(
                    Math.hypot(
                        SnapbackTunerConstants.FrontLeft.LocationX,
                        SnapbackTunerConstants.FrontRight.LocationY),
                    Math.hypot(
                        SnapbackTunerConstants.FrontRight.LocationX,
                        SnapbackTunerConstants.FrontRight.LocationY)),
                Math.max(
                    Math.hypot(
                        SnapbackTunerConstants.BackLeft.LocationX,
                        SnapbackTunerConstants.BackLeft.LocationY),
                    Math.hypot(
                        SnapbackTunerConstants.BackRight.LocationX,
                        SnapbackTunerConstants.BackRight.LocationY)));
        MAX_ANGULAR_VELOCITY = (MAX_LINEAR_VELOCITY / DRIVE_BASE_RADIUS);
        KINEMATICS =
            new SwerveDriveKinematics(
                new Translation2d[] {
                  new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
                  new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
                  new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
                  new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
                });

        FRONT_LEFT = SnapbackTunerConstants.FrontLeft;
        FRONT_RIGHT = SnapbackTunerConstants.FrontRight;
        BACK_LEFT = SnapbackTunerConstants.BackLeft;
        BACK_RIGHT = SnapbackTunerConstants.BackRight;

        DRIVER_DEADBAND = 0.025;

        DRIVE_KP = SnapbackTunerConstants.driveGains.kP;
        DRIVE_KD = SnapbackTunerConstants.driveGains.kD;
        TURN_KP = SnapbackTunerConstants.steerGains.kP;
        TURN_KD = SnapbackTunerConstants.steerGains.kD;

        DRIVE_KS = SnapbackTunerConstants.driveGains.kS;
        DRIVE_KV = SnapbackTunerConstants.driveGains.kV;

        AUTO_X_KP = 4.0;
        AUTO_X_KD = 0.0;

        AUTO_Y_KP = 4.0;
        AUTO_Y_KD = 0.0;

        AUTO_THETA_KP = 5.0;
        AUTO_THETA_KD = 0.05;
        break;
      case ROBOT_SIM:
      default:
        CAN_BUS = "";
        GYRO_CAN_ID = 0;
        ODOMETRY_FREQUENCY = 0.0;

        TRACK_WIDTH_X =
            Math.abs(WhiplashTunerConstants.FrontLeft.LocationX)
                + Math.abs(WhiplashTunerConstants.FrontRight.LocationX);
        TRACK_WIDTH_Y =
            Math.abs(WhiplashTunerConstants.FrontLeft.LocationY)
                + Math.abs(WhiplashTunerConstants.BackLeft.LocationY);
        WHEEL_RADIUS = Units.inchesToMeters(2.0);
        MAX_LINEAR_VELOCITY = Units.feetToMeters(15.0);
        DRIVE_BASE_RADIUS =
            Math.max(
                Math.max(
                    Math.hypot(
                        WhiplashTunerConstants.FrontLeft.LocationX,
                        WhiplashTunerConstants.FrontRight.LocationY),
                    Math.hypot(
                        WhiplashTunerConstants.FrontRight.LocationX,
                        WhiplashTunerConstants.FrontRight.LocationY)),
                Math.max(
                    Math.hypot(
                        WhiplashTunerConstants.BackLeft.LocationX,
                        WhiplashTunerConstants.BackLeft.LocationY),
                    Math.hypot(
                        WhiplashTunerConstants.BackRight.LocationX,
                        WhiplashTunerConstants.BackRight.LocationY)));
        MAX_ANGULAR_VELOCITY = (MAX_LINEAR_VELOCITY / DRIVE_BASE_RADIUS) * 0.6;
        KINEMATICS =
            new SwerveDriveKinematics(
                new Translation2d[] {
                  new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
                  new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
                  new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
                  new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
                });

        FRONT_LEFT = WhiplashTunerConstants.FrontLeft;
        FRONT_RIGHT = WhiplashTunerConstants.FrontRight;
        BACK_LEFT = WhiplashTunerConstants.BackLeft;
        BACK_RIGHT = WhiplashTunerConstants.BackRight;

        DRIVER_DEADBAND = 0.025;

        DRIVE_KP = 0.005;
        DRIVE_KD = 0.0;
        TURN_KP = 20.0;
        TURN_KD = 0.0;

        DRIVE_KS = 0.0;
        double DRIVE_KV_ROT = 0.91032; // Same units as TunerConstants: (volt * secs) / rotation
        DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);

        AUTO_X_KP = 4.0;
        AUTO_X_KD = 0.0;

        AUTO_Y_KP = 4.0;
        AUTO_Y_KD = 0.0;

        AUTO_THETA_KP = 5.0;
        AUTO_THETA_KD = 0.05;
        break;
    }

    DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
    TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);
  }
}
