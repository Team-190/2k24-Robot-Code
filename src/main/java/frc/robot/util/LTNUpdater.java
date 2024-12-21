package frc.robot.util;

import choreo.Choreo;
import choreo.auto.AutoFactory.AutoBindings;
import frc.robot.RobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.drive.DriveConstants;
import frc.robot.subsystems.snapback.hood.SnapbackHood;
import frc.robot.subsystems.snapback.hood.SnapbackHoodConstants;
import frc.robot.subsystems.snapback.shooter.SnapbackShooter;
import frc.robot.subsystems.snapback.shooter.SnapbackShooterConstants;
import frc.robot.subsystems.whiplash.arm.WhiplashArm;
import frc.robot.subsystems.whiplash.arm.WhiplashArmConstants;
import frc.robot.subsystems.whiplash.shooter.WhiplashShooter;
import frc.robot.subsystems.whiplash.shooter.WhiplashShooterConstants;

public class LTNUpdater {
  private static final void updateDrive(Drive drive) {
    LoggedTunableNumber.ifChanged(
        drive.hashCode(),
        () -> {
          drive.setPIDGains(
              DriveConstants.GAINS.drive_Kp().get(),
              DriveConstants.GAINS.drive_Kd().get(),
              DriveConstants.GAINS.turn_Kp().get(),
              DriveConstants.GAINS.turn_Kd().get());
          drive.setFFGains(
              DriveConstants.GAINS.drive_Ks().get(), DriveConstants.GAINS.drive_Kv().get());
        },
        DriveConstants.GAINS.drive_Ks(),
        DriveConstants.GAINS.drive_Kv(),
        DriveConstants.GAINS.drive_Kp(),
        DriveConstants.GAINS.drive_Kd(),
        DriveConstants.GAINS.turn_Kp(),
        DriveConstants.GAINS.turn_Kd());

    LoggedTunableNumber.ifChanged(
        drive.hashCode(),
        () -> {
          DriveCommands.setPID(
              DriveConstants.AUTO_ALIGN_GAINS.rotation_Kp().get(),
              DriveConstants.AUTO_ALIGN_GAINS.rotation_Kd().get());
        },
        DriveConstants.AUTO_ALIGN_GAINS.rotation_Kp(),
        DriveConstants.AUTO_ALIGN_GAINS.rotation_Kd());

    LoggedTunableNumber.ifChanged(
        drive.hashCode(),
        () -> {
          drive
              .getAutoController()
              .setPID(
                  DriveConstants.AUTO_ALIGN_GAINS.translation_Kp().get(),
                  DriveConstants.AUTO_ALIGN_GAINS.translation_Kd().get(),
                  DriveConstants.AUTO_ALIGN_GAINS.rotation_Kp().get(),
                  DriveConstants.AUTO_ALIGN_GAINS.rotation_Kd().get());
          drive.setAutoFactory(
              Choreo.createAutoFactory(
                  RobotState::getRobotPose,
                  drive.getAutoController(),
                  AllianceFlipUtil::shouldFlip,
                  drive,
                  new AutoBindings()));
        },
        DriveConstants.AUTO_ALIGN_GAINS.translation_Kp(),
        DriveConstants.AUTO_ALIGN_GAINS.translation_Kd(),
        DriveConstants.AUTO_ALIGN_GAINS.rotation_Kp(),
        DriveConstants.AUTO_ALIGN_GAINS.rotation_Kd());
  }

  private static final void updateWhiplashArm(WhiplashArm whiplashArm) {
    LoggedTunableNumber.ifChanged(
        whiplashArm.hashCode(),
        () -> {
          whiplashArm.setPID(
              WhiplashArmConstants.GAINS.kp().get(), WhiplashArmConstants.GAINS.kd().get());
          whiplashArm.setFeedforward(
              WhiplashArmConstants.GAINS.ks().get(),
              WhiplashArmConstants.GAINS.kg().get(),
              WhiplashArmConstants.GAINS.kv().get());
          whiplashArm.setProfile(
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

  private static final void updateWhiplashShooter(WhiplashShooter whiplashShooter) {
    LoggedTunableNumber.ifChanged(
        whiplashShooter.hashCode(),
        () -> {
          whiplashShooter.setPID(
              WhiplashShooterConstants.GAINS.kp().get(), WhiplashShooterConstants.GAINS.kd().get());
          whiplashShooter.setFeedforward(
              WhiplashShooterConstants.GAINS.ks().get(),
              WhiplashShooterConstants.GAINS.kv().get(),
              WhiplashShooterConstants.GAINS.ka().get());
          whiplashShooter.setProfile(
              WhiplashShooterConstants.CONSTRAINTS.maxAccelerationRadiansPerSecondSquared().get(),
              WhiplashShooterConstants.CONSTRAINTS.goalToleranceRadiansPerSecond().get());
        },
        WhiplashShooterConstants.GAINS.kp(),
        WhiplashShooterConstants.GAINS.kd(),
        WhiplashShooterConstants.GAINS.ks(),
        WhiplashShooterConstants.GAINS.kv(),
        WhiplashShooterConstants.GAINS.ka(),
        WhiplashShooterConstants.CONSTRAINTS.maxAccelerationRadiansPerSecondSquared(),
        WhiplashShooterConstants.CONSTRAINTS.goalToleranceRadiansPerSecond());
  }

  private static final void updateSnapbackHood(SnapbackHood snapbackHood) {
    LoggedTunableNumber.ifChanged(
        snapbackHood.hashCode(),
        () -> {
          snapbackHood.setPID(
              SnapbackHoodConstants.GAINS.kp().get(), SnapbackHoodConstants.GAINS.kd().get());
          snapbackHood.setFeedforward(
              SnapbackHoodConstants.GAINS.ks().get(),
              SnapbackHoodConstants.GAINS.kv().get(),
              SnapbackHoodConstants.GAINS.ka().get());
          snapbackHood.setProfile(
              SnapbackHoodConstants.CONSTRAINTS.maxVelocityRadiansPerSecond().get(),
              SnapbackHoodConstants.CONSTRAINTS.maxAccelerationRadiansPerSecondSqaured().get(),
              SnapbackHoodConstants.CONSTRAINTS.goalToleranceRadians().get());
        },
        SnapbackHoodConstants.GAINS.kp(),
        SnapbackHoodConstants.GAINS.kd(),
        SnapbackHoodConstants.GAINS.ks(),
        SnapbackHoodConstants.GAINS.kv(),
        SnapbackHoodConstants.GAINS.ka(),
        SnapbackHoodConstants.CONSTRAINTS.maxVelocityRadiansPerSecond(),
        SnapbackHoodConstants.CONSTRAINTS.maxAccelerationRadiansPerSecondSqaured(),
        SnapbackHoodConstants.CONSTRAINTS.goalToleranceRadians());
  }

  private static final void updateSnapbackShooter(SnapbackShooter snapbackShooter) {
    LoggedTunableNumber.ifChanged(
        snapbackShooter.hashCode(),
        () -> {
          snapbackShooter.setPID(
              SnapbackShooterConstants.GAINS.kp().get(), SnapbackShooterConstants.GAINS.kd().get());
          snapbackShooter.setFeedforward(
              SnapbackShooterConstants.GAINS.ks().get(),
              SnapbackShooterConstants.GAINS.kv().get(),
              SnapbackShooterConstants.GAINS.ka().get());
          snapbackShooter.setProfile(
              SnapbackShooterConstants.CONSTRAINTS.maxAccelerationRadiansPerSecondSquared().get(),
              SnapbackShooterConstants.CONSTRAINTS.goalToleranceRadiansPerSecond().get());
        },
        SnapbackShooterConstants.GAINS.kp(),
        SnapbackShooterConstants.GAINS.kd(),
        SnapbackShooterConstants.GAINS.ks(),
        SnapbackShooterConstants.GAINS.kv(),
        SnapbackShooterConstants.GAINS.ka(),
        SnapbackShooterConstants.CONSTRAINTS.maxAccelerationRadiansPerSecondSquared(),
        SnapbackShooterConstants.CONSTRAINTS.goalToleranceRadiansPerSecond());
  }

  public static final void updateWhiplash(
      Drive drive, WhiplashArm whiplashArm, WhiplashShooter whiplashShooter) {
    updateDrive(drive);
    updateWhiplashArm(whiplashArm);
    updateWhiplashShooter(whiplashShooter);
  }

  public static final void updateSnapback(
      Drive drive, SnapbackHood snapbackHood, SnapbackShooter snapbackShooter) {
    updateDrive(drive);
    updateSnapbackHood(snapbackHood);
    updateSnapbackShooter(snapbackShooter);
  }
}
