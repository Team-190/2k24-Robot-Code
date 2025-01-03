package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.snapback.hood.SnapbackHood;
import frc.robot.subsystems.snapback.hood.SnapbackHoodConstants.SnapbackHoodGoal;
import frc.robot.subsystems.snapback.intake.SnapbackIntake;
import frc.robot.subsystems.snapback.shooter.SnapbackShooter;
import frc.robot.subsystems.snapback.shooter.SnapbackShooterConstants.SnapbackShooterGoal;
import frc.robot.subsystems.whiplash.arm.WhiplashArm;
import frc.robot.subsystems.whiplash.arm.WhiplashArmConstants.WhiplashArmGoal;
import frc.robot.subsystems.whiplash.intake.WhiplashIntake;
import frc.robot.subsystems.whiplash.shooter.WhiplashShooter;
import frc.robot.subsystems.whiplash.shooter.WhiplashShooterConstants.WhiplashShooterGoal;
import frc.robot.util.AllianceFlipUtil;

public class CompositeCommands {
  // Shared
  public static final Command resetHeading(Drive drive) {
    return Commands.runOnce(
            () -> {
              RobotState.resetRobotPose(
                  new Pose2d(
                      RobotState.getRobotPose().getTranslation(),
                      AllianceFlipUtil.apply(new Rotation2d())));
            })
        .ignoringDisable(true);
  }

  // Snapback
  public static final Command collect(SnapbackIntake intake) {
    return Commands.sequence(intake.intake());
  }

  public static final Command eject(SnapbackIntake intake) {
    return Commands.sequence(intake.outtake());
  }

  public static final Command shootSpeaker(
      Drive drive,
      SnapbackIntake intake,
      SnapbackHood hood,
      SnapbackShooter shooter,
      XboxController driver) {
    return Commands.sequence(
            Commands.parallel(
                    shooter.setGoal(SnapbackShooterGoal.SPEAKER),
                    hood.setGoal(SnapbackHoodGoal.SPEAKER))
                .until(
                    () ->
                        shooter.atGoal()
                            && hood.atGoal()
                            && DriveCommands.getAimController().atSetpoint()
                            && drive.getYawVelocity() <= Units.degreesToRadians(1)),
            Commands.waitSeconds(0.125),
            intake.shoot(),
            Commands.runOnce(() -> driver.setRumble(RumbleType.kBothRumble, 1)),
            Commands.waitSeconds(1))
        .finallyDo(() -> driver.setRumble(RumbleType.kBothRumble, 0));
  }

  public static final Command shootSpeakerAuto(
      Drive drive, SnapbackIntake intake, SnapbackHood hood, SnapbackShooter shooter) {
    return Commands.sequence(
        Commands.parallel(
            shooter.setGoal(SnapbackShooterGoal.SPEAKER), hood.setGoal(SnapbackHoodGoal.SPEAKER)),
        Commands.waitUntil(
            () ->
                shooter.atGoal() && hood.atGoal() && DriveCommands.getAimController().atSetpoint()),
        Commands.runOnce(() -> drive.stop()),
        Commands.waitSeconds(0.125),
        intake.shoot());
  }

  public static final Command shootFeed(
      SnapbackIntake intake, SnapbackHood hood, SnapbackShooter shooter) {
    return Commands.sequence(
        Commands.parallel(
            shooter.setGoal(SnapbackShooterGoal.FEED), hood.setGoal(SnapbackHoodGoal.FEED)),
        Commands.waitUntil(() -> shooter.atGoal() && hood.atGoal()),
        intake.shoot());
  }

  // Whiplash
  public static final Command collect(WhiplashIntake intake, WhiplashArm arm) {
    return Commands.sequence(
        arm.setGoal(WhiplashArmGoal.INTAKE),
        Commands.waitUntil(() -> arm.atGoal()),
        intake.intake());
  }

  public static final Command eject(WhiplashIntake intake, WhiplashArm arm) {
    return Commands.sequence(
        arm.setGoal(WhiplashArmGoal.EJECT), Commands.waitUntil(() -> arm.atGoal()), intake.eject());
  }

  public static final Command shootSpeaker(
      Drive drive,
      WhiplashIntake intake,
      WhiplashArm arm,
      WhiplashShooter shooter,
      XboxController driver) {
    return Commands.sequence(
            Commands.parallel(
                    shooter.setGoal(WhiplashShooterGoal.SPEAKER),
                    arm.setGoal(WhiplashArmGoal.SPEAKER))
                .until(
                    () ->
                        shooter.atGoal()
                            && arm.atGoal()
                            && DriveCommands.getAimController().atSetpoint()
                            && drive.getYawVelocity() <= Units.degreesToRadians(1)),
            Commands.waitSeconds(0.125),
            intake.shoot(),
            Commands.either(
                Commands.sequence(
                    intake.intake(),
                    Commands.parallel(
                            shooter.setGoal(WhiplashShooterGoal.SPEAKER),
                            arm.setGoal(WhiplashArmGoal.SPEAKER))
                        .until(
                            () ->
                                shooter.atGoal()
                                    && arm.atGoal()
                                    && DriveCommands.getAimController().atSetpoint()),
                    Commands.waitSeconds(0.125),
                    intake.shoot(),
                    arm.setGoal(WhiplashArmGoal.STOW)),
                arm.setGoal(WhiplashArmGoal.STOW),
                () -> intake.hasNoteStaged()),
            Commands.runOnce(() -> driver.setRumble(RumbleType.kBothRumble, 1)),
            Commands.waitSeconds(1))
        .finallyDo(() -> driver.setRumble(RumbleType.kBothRumble, 0));
  }

  public static final Command shootSpeakerAuto(
      Drive drive, WhiplashIntake intake, WhiplashArm arm, WhiplashShooter shooter) {
    return Commands.sequence(
        Commands.deadline(
            shooter.setGoal(WhiplashShooterGoal.SPEAKER), arm.setGoal(WhiplashArmGoal.SPEAKER)),
        Commands.waitUntil(
            () ->
                shooter.atGoal() && arm.atGoal() && DriveCommands.getAimController().atSetpoint()),
        Commands.runOnce(() -> drive.stop()),
        Commands.waitSeconds(0.125),
        intake.shoot());
  }

  public static final Command shootSubwoofer(
      WhiplashIntake intake, WhiplashArm arm, WhiplashShooter shooter) {
    return Commands.sequence(
        Commands.parallel(
            shooter.setGoal(WhiplashShooterGoal.SPEAKER), arm.setGoal(WhiplashArmGoal.SUBWOOFER)),
        Commands.waitUntil(() -> shooter.atGoal() && arm.atGoal()),
        intake.shoot(),
        Commands.either(
            Commands.sequence(
                CompositeCommands.collect(intake, arm),
                Commands.parallel(
                    shooter.setGoal(WhiplashShooterGoal.SPEAKER),
                    arm.setGoal(WhiplashArmGoal.SUBWOOFER)),
                Commands.waitUntil(() -> shooter.atGoal() && arm.atGoal()),
                intake.shoot(),
                arm.setGoal(WhiplashArmGoal.STOW)),
            arm.setGoal(WhiplashArmGoal.STOW),
            () -> intake.hasNoteStaged()));
  }

  public static final Command shootAmp(
      WhiplashIntake intake, WhiplashArm arm, WhiplashShooter shooter) {
    return Commands.sequence(
        Commands.parallel(
            shooter.setGoal(WhiplashShooterGoal.AMP), arm.setGoal(WhiplashArmGoal.PREAMP)),
        Commands.waitUntil(() -> shooter.atGoal() && arm.atGoal()),
        Commands.waitSeconds(0.125),
        Commands.parallel(intake.shoot(), arm.setGoal(WhiplashArmGoal.AMP)),
        Commands.waitSeconds(0.5),
        Commands.either(
            Commands.sequence(
                CompositeCommands.collect(intake, arm),
                Commands.parallel(
                    shooter.setGoal(WhiplashShooterGoal.AMP), arm.setGoal(WhiplashArmGoal.PREAMP)),
                Commands.waitUntil(() -> shooter.atGoal() && arm.atGoal()),
                Commands.waitSeconds(0.125),
                Commands.parallel(intake.shoot(), arm.setGoal(WhiplashArmGoal.AMP)),
                Commands.waitSeconds(0.5),
                arm.setGoal(WhiplashArmGoal.STOW)),
            arm.setGoal(WhiplashArmGoal.STOW),
            () -> intake.hasNoteStaged()));
  }

  public static final Command shootFeed(
      WhiplashIntake intake, WhiplashArm arm, WhiplashShooter shooter) {
    return Commands.sequence(
        Commands.parallel(
            shooter.setGoal(WhiplashShooterGoal.FEED), arm.setGoal(WhiplashArmGoal.FEED)),
        Commands.waitUntil(() -> shooter.atGoal() && arm.atGoal()),
        intake.shoot(),
        Commands.either(
            Commands.sequence(
                CompositeCommands.collect(intake, arm),
                Commands.parallel(
                    shooter.setGoal(WhiplashShooterGoal.FEED), arm.setGoal(WhiplashArmGoal.FEED)),
                Commands.waitUntil(() -> shooter.atGoal() && arm.atGoal()),
                intake.shoot(),
                arm.setGoal(WhiplashArmGoal.STOW)),
            arm.setGoal(WhiplashArmGoal.STOW),
            () -> intake.hasNoteStaged()));
  }
}
