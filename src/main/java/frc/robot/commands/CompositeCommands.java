package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionMode;

public class CompositeCommands {
  public static final Command getTrackNoteCenterCommand(
      Drive drive, Intake intake, Feeder feeder, Vision noteVision) {
    return DriveCommands.moveTowardsTarget(
            drive, noteVision, FieldConstants.fieldLength / 2.0, VisionMode.Notes)
        .alongWith(intake.runVoltage())
        .alongWith(feeder.intake());
  }

  public static final Command getTrackNoteSpikeCommand(
      Drive drive, Intake intake, Feeder feeder, Vision noteVision) {
    return DriveCommands.moveTowardsTarget(
            drive, noteVision, FieldConstants.startingLineX + 0.5, VisionMode.Notes)
        .alongWith(intake.runVoltage())
        .alongWith(feeder.intake());
  }

  public static final Command getTrackSpeakerFarCommand(Drive drive, Vision aprilTagVision) {
    return DriveCommands.moveTowardsTarget(drive, aprilTagVision, 3.75, VisionMode.AprilTags);
  }

  public static final Command getTrackSpeakerCloseCommand(Drive drive, Vision aprilTagVision) {
    return DriveCommands.moveTowardsTarget(
        drive, aprilTagVision, FieldConstants.startingLineX - 0.25, VisionMode.AprilTags);
  }

  public static final Command getCollectCommand(Intake intake, Feeder feeder) {
    return intake.collect().alongWith(feeder.intake());
  }

  public static final Command getAccelerateShooterCommand(Shooter shooter) {
    return shooter.runVelocity();
  }

  public static final Command getShootCommand(Feeder feeder) {
    return feeder.shoot();
  }
}
