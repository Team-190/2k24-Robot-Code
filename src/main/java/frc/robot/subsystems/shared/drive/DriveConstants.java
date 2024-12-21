package frc.robot.subsystems.shared.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class DriveConstants {
  public static final DriveConfig DRIVE_CONFIG;

  public static final SwerveModuleConstants FRONT_LEFT;
  public static final SwerveModuleConstants FRONT_RIGHT;
  public static final SwerveModuleConstants BACK_LEFT;
  public static final SwerveModuleConstants BACK_RIGHT;

  public static final Gains GAINS;
  public static final AutoAlignGains AUTO_ALIGN_GAINS;

  public static final double ODOMETRY_FREQUENCY;
  public static final double DRIVER_DEADBAND;

  static {
    switch (Constants.ROBOT) {
      case WHIPLASH:
      case WHIPLASH_SIM:
      default:
        DRIVE_CONFIG =
            new DriveConfig(
                TunerConstantsWhiplash.DrivetrainConstants.CANBusName,
                TunerConstantsWhiplash.DrivetrainConstants.Pigeon2Id,
                TunerConstantsWhiplash.FrontLeft.WheelRadius,
                Math.abs(TunerConstantsWhiplash.FrontLeft.LocationX)
                    + Math.abs(TunerConstantsWhiplash.FrontRight.LocationX),
                Math.abs(TunerConstantsWhiplash.FrontLeft.LocationY)
                    + Math.abs(TunerConstantsWhiplash.BackLeft.LocationY),
                TunerConstantsWhiplash.kSpeedAt12Volts.in(MetersPerSecond),
                DCMotor.getKrakenX60Foc(1),
                DCMotor.getKrakenX60Foc(1));

        FRONT_LEFT = TunerConstantsWhiplash.FrontLeft;
        FRONT_RIGHT = TunerConstantsWhiplash.FrontRight;
        BACK_LEFT = TunerConstantsWhiplash.BackLeft;
        BACK_RIGHT = TunerConstantsWhiplash.BackRight;

        GAINS =
            new Gains(
                new LoggedTunableNumber("Drive/Drive KS", 0.0),
                new LoggedTunableNumber("Drive/Drive KV", 0.0),
                new LoggedTunableNumber("Drive/Drive KP", 0.0),
                new LoggedTunableNumber("Drive/Drive KD", 0.0),
                new LoggedTunableNumber("Drive/Turn KP", 0.0),
                new LoggedTunableNumber("Drive/Turn KD", 0.0));
        AUTO_ALIGN_GAINS =
            new AutoAlignGains(
                new LoggedTunableNumber("Drive/Translation KP", 4.0),
                new LoggedTunableNumber("Drive/Translation KD", 0.0),
                new LoggedTunableNumber("Drive/Rotation KP", 5.0),
                new LoggedTunableNumber("Drive/Rotation KD", 0.05));

        ODOMETRY_FREQUENCY = 250.0;
        DRIVER_DEADBAND = 0.025;
        break;
      case SNAPBACK:
      case SNAPBACK_SIM:
        DRIVE_CONFIG =
            new DriveConfig(
                TunerConstantsSnapback.DrivetrainConstants.CANBusName,
                TunerConstantsSnapback.DrivetrainConstants.Pigeon2Id,
                TunerConstantsSnapback.FrontLeft.WheelRadius,
                Math.abs(TunerConstantsSnapback.FrontLeft.LocationX)
                    + Math.abs(TunerConstantsSnapback.FrontRight.LocationX),
                Math.abs(TunerConstantsSnapback.FrontLeft.LocationY)
                    + Math.abs(TunerConstantsSnapback.BackLeft.LocationY),
                TunerConstantsSnapback.kSpeedAt12Volts.in(MetersPerSecond),
                DCMotor.getKrakenX60Foc(1),
                DCMotor.getKrakenX60Foc(1));

        FRONT_LEFT = TunerConstantsSnapback.FrontLeft;
        FRONT_RIGHT = TunerConstantsSnapback.FrontRight;
        BACK_LEFT = TunerConstantsSnapback.BackLeft;
        BACK_RIGHT = TunerConstantsSnapback.BackRight;

        GAINS =
            new Gains(
                new LoggedTunableNumber("Drive/Drive KS"),
                new LoggedTunableNumber("Drive/Drive KV"),
                new LoggedTunableNumber("Drive/Drive KP"),
                new LoggedTunableNumber("Drive/Drive KD"),
                new LoggedTunableNumber("Drive/Turn KP"),
                new LoggedTunableNumber("Drive/Turn KD"));
        AUTO_ALIGN_GAINS =
            new AutoAlignGains(
                new LoggedTunableNumber("Drive/Translation KP", 4.0),
                new LoggedTunableNumber("Drive/Translation KD", 0.0),
                new LoggedTunableNumber("Drive/Rotation KP", 5.0),
                new LoggedTunableNumber("Drive/Rotation KD", 0.05));
        ODOMETRY_FREQUENCY = 250.0;
        DRIVER_DEADBAND = 0.025;
        break;
    }
  }

  public record DriveConfig(
      String canBus,
      int pigeon2Id,
      double wheelRadiusMeters,
      double trackWidthXMeters,
      double trackWidthYMeters,
      double maxLinearVelocityMetersPerSecond,
      DCMotor driveModel,
      DCMotor turnModel) {
    public double driveBaseRadius() {
      return Math.hypot(trackWidthXMeters / 2.0, trackWidthYMeters / 2.0);
    }

    public double maxAngularVelocity() {
      return maxLinearVelocityMetersPerSecond / driveBaseRadius();
    }

    public Translation2d[] getModuleTranslations() {
      return new Translation2d[] {
        new Translation2d(
            TunerConstantsWhiplash.FrontLeft.LocationX, TunerConstantsWhiplash.FrontLeft.LocationY),
        new Translation2d(
            TunerConstantsWhiplash.FrontRight.LocationX,
            TunerConstantsWhiplash.FrontRight.LocationY),
        new Translation2d(
            TunerConstantsWhiplash.BackLeft.LocationX, TunerConstantsWhiplash.BackLeft.LocationY),
        new Translation2d(
            TunerConstantsWhiplash.BackRight.LocationX, TunerConstantsWhiplash.BackRight.LocationY)
      };
    }

    public SwerveDriveKinematics kinematics() {
      return new SwerveDriveKinematics(getModuleTranslations());
    }
  }

  public record Gains(
      LoggedTunableNumber driveKs,
      LoggedTunableNumber driveKv,
      LoggedTunableNumber driveKp,
      LoggedTunableNumber driveKd,
      LoggedTunableNumber turnKp,
      LoggedTunableNumber turnKd) {}

  public record AutoAlignGains(
      LoggedTunableNumber translation_Kp,
      LoggedTunableNumber translation_Kd,
      LoggedTunableNumber rotation_Kp,
      LoggedTunableNumber rotation_Kd) {}
}
