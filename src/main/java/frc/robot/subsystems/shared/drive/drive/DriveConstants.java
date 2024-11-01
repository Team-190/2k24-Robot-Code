package frc.robot.subsystems.shared.drive.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public final class DriveConstants {
  public static final double TRACK_WIDTH_X;
  public static final double TRACK_WIDTH_Y;
  public static final double MAX_LINEAR_VELOCITY;
  public static final double MAX_ANGULAR_VELOCITY;
  public static final double DRIVE_BASE_RADIUS;
  public static final SwerveDriveKinematics KINEMATICS;
  public static final String CANIVORE;
  public static final int PIGEON_2_DEVICE_ID;
  public static final Matrix<N3, N1> ODOMETRY_STANDARD_DEVIATIONS;
  public static final double DRIVER_DEADBAND;
  public static final Lock ODOMETRY_LOCK;
  public static final double AUTO_AIM_FIELD_VELOCITY_DEADBAND;
  public static final double ROBOT_MASS_KG;
  public static final double ROBOT_MOMENT_OF_INERTIA;

  public static final LoggedTunableNumber AUTO_X_KP;
  public static final LoggedTunableNumber AUTO_Y_KP;
  public static final LoggedTunableNumber AUTO_THETA_KP;
  public static final LoggedTunableNumber AUTO_X_KD;
  public static final LoggedTunableNumber AUTO_Y_KD;
  public static final LoggedTunableNumber AUTO_THETA_KD;

  static {
    AUTO_X_KP = new LoggedTunableNumber("Drive/Auto X KP");
    AUTO_Y_KP = new LoggedTunableNumber("Drive/Auto Y KP");
    AUTO_THETA_KP = new LoggedTunableNumber("Drive/Auto Theta KP");
    AUTO_X_KD = new LoggedTunableNumber("Drive/Auto X KD");
    AUTO_Y_KD = new LoggedTunableNumber("Drive/Auto Y KD");
    AUTO_THETA_KD = new LoggedTunableNumber("Drive/Auto Theta KD");
    switch (Constants.ROBOT) {
      case WHIPLASH:
      case ROBOT_SIM:
      default:
        TRACK_WIDTH_X = Units.inchesToMeters(20.75);
        TRACK_WIDTH_Y = Units.inchesToMeters(20.75);
        MAX_LINEAR_VELOCITY = Units.feetToMeters(15.0);
        DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2, TRACK_WIDTH_Y / 2);
        MAX_ANGULAR_VELOCITY = (MAX_LINEAR_VELOCITY / DRIVE_BASE_RADIUS) * 0.6;
        KINEMATICS =
            new SwerveDriveKinematics(
                new Translation2d[] {
                  new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
                  new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
                  new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
                  new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
                });
        CANIVORE = "drive";
        PIGEON_2_DEVICE_ID = 1;
        ODOMETRY_STANDARD_DEVIATIONS = VecBuilder.fill(0.1, 0.1, 0.1);
        DRIVER_DEADBAND = 0.025;
        ODOMETRY_LOCK = new ReentrantLock();
        AUTO_AIM_FIELD_VELOCITY_DEADBAND = 0.1;

        AUTO_X_KP.initDefault(4.0);
        AUTO_Y_KP.initDefault(4.0);
        AUTO_THETA_KP.initDefault(5.0);

        AUTO_X_KD.initDefault(0.0);
        AUTO_Y_KD.initDefault(0.0);
        AUTO_THETA_KD.initDefault(0.05);

        ROBOT_MASS_KG = 51.2559;
        ROBOT_MOMENT_OF_INERTIA = 3.72559543;
        break;
    }
  }
}
