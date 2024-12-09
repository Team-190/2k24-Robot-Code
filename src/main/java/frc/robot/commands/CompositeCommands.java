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
import frc.robot.subsystems.snapback.intake.SnapbackIntake;
import frc.robot.subsystems.snapback.shooter.SnapbackShooter;
import frc.robot.subsystems.snapback.shooter.SnapbackShooterConstants.Goal;
import frc.robot.subsystems.whiplash.arm.WhiplashArm;
import frc.robot.subsystems.whiplash.intake.WhiplashIntake;
import frc.robot.subsystems.whiplash.shooter.WhiplashShooter;
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
            Commands.parallel(shooter.setGoal(Goal.SPEAKER), hood.setSpeaker())
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
        Commands.parallel(shooter.setGoal(Goal.SPEAKER), hood.setSpeaker()),
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
        Commands.parallel(shooter.setGoal(Goal.FEED), hood.setFeed()),
        Commands.waitUntil(() -> shooter.atGoal() && hood.atGoal()),
        intake.shoot());
  }

  // Whiplash
  public static final Command collect(WhiplashIntake intake, WhiplashArm arm) {
    return Commands.sequence(
        arm.intakeAngle(), Commands.waitUntil(() -> arm.atSetpoint()), intake.intake());
  }

  public static final Command eject(WhiplashIntake intake, WhiplashArm arm) {
    return Commands.sequence(
        arm.ejectCommand(), Commands.waitUntil(() -> arm.atSetpoint()), intake.eject());
  }

  public static final Command shootSpeaker(
      Drive drive,
      WhiplashIntake intake,
      WhiplashArm arm,
      WhiplashShooter shooter,
      XboxController driver) {
    return Commands.sequence(
            Commands.parallel(shooter.setSpeakerVelocity(), arm.shootAngle())
                .until(
                    () ->
                        shooter.atSetPoint()
                            && arm.atSetpoint()
                            && DriveCommands.getAimController().atSetpoint()
                            && drive.getYawVelocity() <= Units.degreesToRadians(1)),
            Commands.waitSeconds(0.125),
            intake.shoot(),
            Commands.either(
                Commands.sequence(
                    intake.intake(),
                    Commands.parallel(shooter.setSpeakerVelocity(), arm.shootAngle())
                        .until(
                            () ->
                                shooter.atSetPoint()
                                    && arm.atSetpoint()
                                    && DriveCommands.getAimController().atSetpoint()),
                    Commands.waitSeconds(0.125),
                    intake.shoot(),
                    arm.stowAngle()),
                arm.stowAngle(),
                () -> intake.hasNoteStaged()),
            Commands.runOnce(() -> driver.setRumble(RumbleType.kBothRumble, 1)),
            Commands.waitSeconds(1))
        .finallyDo(() -> driver.setRumble(RumbleType.kBothRumble, 0));
  }

  public static final Command shootSpeakerAuto(
      Drive drive, WhiplashIntake intake, WhiplashArm arm, WhiplashShooter shooter) {
    return Commands.sequence(
        Commands.deadline(shooter.setSpeakerVelocity(), arm.shootAngle()),
        Commands.waitUntil(
            () ->
                shooter.atSetPoint()
                    && arm.atSetpoint()
                    && DriveCommands.getAimController().atSetpoint()),
        Commands.runOnce(() -> drive.stop()),
        Commands.waitSeconds(0.125),
        intake.shoot());
  }

  public static final Command shootSubwoofer(
      WhiplashIntake intake, WhiplashArm arm, WhiplashShooter shooter) {
    return Commands.sequence(
        Commands.parallel(shooter.setSpeakerVelocity(), arm.subwooferAngle()),
        Commands.waitUntil(() -> shooter.atSetPoint() && arm.atSetpoint()),
        intake.shoot(),
        Commands.either(
            Commands.sequence(
                CompositeCommands.collect(intake, arm),
                Commands.parallel(shooter.setSpeakerVelocity(), arm.subwooferAngle()),
                Commands.waitUntil(() -> shooter.atSetPoint() && arm.atSetpoint()),
                intake.shoot(),
                arm.stowAngle()),
            arm.stowAngle(),
            () -> intake.hasNoteStaged()));
  }

  public static final Command shootAmp(
      WhiplashIntake intake, WhiplashArm arm, WhiplashShooter shooter) {
    return Commands.sequence(
        Commands.parallel(shooter.setAmpVelocity(), arm.preAmpAngle()),
        Commands.waitUntil(() -> shooter.atSetPoint() && arm.atSetpoint()),
        Commands.waitSeconds(0.125),
        Commands.parallel(intake.shoot(), arm.ampAngle()),
        Commands.waitSeconds(0.5),
        Commands.either(
            Commands.sequence(
                CompositeCommands.collect(intake, arm),
                Commands.parallel(shooter.setAmpVelocity(), arm.preAmpAngle()),
                Commands.waitUntil(() -> shooter.atSetPoint() && arm.atSetpoint()),
                Commands.waitSeconds(0.125),
                Commands.parallel(intake.shoot(), arm.ampAngle()),
                Commands.waitSeconds(0.5),
                arm.stowAngle()),
            arm.stowAngle(),
            () -> intake.hasNoteStaged()));
  }

  public static final Command shootFeed(
      WhiplashIntake intake, WhiplashArm arm, WhiplashShooter shooter) {
    return Commands.sequence(
        Commands.parallel(shooter.setFeedVelocity(), arm.feedAngle()),
        Commands.waitUntil(() -> shooter.atSetPoint() && arm.atSetpoint()),
        intake.shoot(),
        Commands.either(
            Commands.sequence(
                CompositeCommands.collect(intake, arm),
                Commands.parallel(shooter.setFeedVelocity(), arm.feedAngle()),
                Commands.waitUntil(() -> shooter.atSetPoint() && arm.atSetpoint()),
                intake.shoot(),
                arm.stowAngle()),
            arm.stowAngle(),
            () -> intake.hasNoteStaged()));
  }
}
