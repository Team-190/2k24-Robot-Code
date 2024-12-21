package frc.robot.util;

import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.drive.DriveConstants;
import frc.robot.subsystems.whiplash.arm.WhiplashArm;
import frc.robot.subsystems.whiplash.arm.WhiplashArmConstants;
import frc.robot.subsystems.whiplash.intake.WhiplashIntake;
import frc.robot.subsystems.whiplash.shooter.WhiplashShooter;
import frc.robot.subsystems.whiplash.shooter.WhiplashShooterConstants;

public class LTNUpdater {
  private static final void updateDrive(Drive drive) {
    LoggedTunableNumber.ifChanged(
        drive.hashCode(),
        () -> {
          drive.setPIDGains(
              DriveConstants.GAINS.driveKp().get(),
              DriveConstants.GAINS.driveKd().get(),
              DriveConstants.GAINS.turnKp().get(),
              DriveConstants.GAINS.turnKd().get());
          drive.setFFGains(
              DriveConstants.GAINS.driveKs().get(), DriveConstants.GAINS.driveKv().get());
        },
        DriveConstants.GAINS.driveKs(),
        DriveConstants.GAINS.driveKv(),
        DriveConstants.GAINS.driveKp(),
        DriveConstants.GAINS.driveKd(),
        DriveConstants.GAINS.turnKp(),
        DriveConstants.GAINS.turnKd(),
        DriveConstants.AUTO_ALIGN_GAINS.translation_Kp(),
        DriveConstants.AUTO_ALIGN_GAINS.translation_Kd(),
        DriveConstants.AUTO_ALIGN_GAINS.rotation_Kp(),
        DriveConstants.AUTO_ALIGN_GAINS.rotation_Kd());
  }

  private static final void updateWhiplashArm(WhiplashArm whiplashArm) {
    LoggedTunableNumber.ifChanged(
        whiplashArm.hashCode(),
        () -> {
          whiplashArm.setPIDGains(
              WhiplashArmConstants.GAINS.kp().get(), WhiplashArmConstants.GAINS.kd().get());
          whiplashArm.setFeedforward(
              WhiplashArmConstants.GAINS.ks().get(),
              WhiplashArmConstants.GAINS.kg().get(),
              WhiplashArmConstants.GAINS.kv().get());
          whiplashArm.setConstraints(
              WhiplashArmConstants.CONSTRAINTS.maxVelocityRadiansPerSecond().get(),
              WhiplashArmConstants.CONSTRAINTS.maxAccelerationRadiansPerSecondSqaured().get(),
              WhiplashArmConstants.CONSTRAINTS.goalToleranceRadians().get());
        },
        WhiplashArmConstants.GAINS.kp(),
        WhiplashArmConstants.GAINS.kd(),
        WhiplashArmConstants.GAINS.ks(),
        WhiplashArmConstants.GAINS.kg(),
        WhiplashArmConstants.GAINS.kv(),
        WhiplashArmConstants.CONSTRAINTS.maxVelocityRadiansPerSecond(),
        WhiplashArmConstants.CONSTRAINTS.maxAccelerationRadiansPerSecondSqaured(),
        WhiplashArmConstants.CONSTRAINTS.goalToleranceRadians());
  }

  private static final void updateWhiplashIntake(WhiplashIntake whiplashIntake) {}

  private static final void updateWhiplashShooter(WhiplashShooter whiplashShooter) {
    LoggedTunableNumber.ifChanged(
        whiplashShooter.hashCode(),
        () -> {
          whiplashShooter.setPID(
              WhiplashShooterConstants.KP.get(), WhiplashShooterConstants.KD.get());
          whiplashShooter.setFeedforward(
              WhiplashShooterConstants.KS.get(),
              WhiplashShooterConstants.KV.get(),
              WhiplashShooterConstants.KA.get());
          whiplashShooter.setProfile(
              WhiplashShooterConstants.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED.get());
        },
        WhiplashShooterConstants.KP,
        WhiplashShooterConstants.KD,
        WhiplashShooterConstants.KS,
        WhiplashShooterConstants.KV,
        WhiplashShooterConstants.KA,
        WhiplashShooterConstants.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
  }

  public static final void updateWhiplash(
      Drive drive,
      WhiplashArm whiplashArm,
      WhiplashIntake whiplashIntake,
      WhiplashShooter whiplashShooter) {
    updateDrive(drive);
    updateWhiplashArm(whiplashArm);
    updateWhiplashIntake(whiplashIntake);
    updateWhiplashShooter(whiplashShooter);
  }
}
