package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionMode;
import java.util.Optional;

public class CompositeCommands {
  public static final Command getTrackNoteCenterCommand(
      Drive drive, Intake intake, Feeder feeder, Vision noteVision) {
    return (DriveCommands.moveTowardsTarget(
                drive, noteVision, FieldConstants.fieldLength / 2.0, VisionMode.Notes)
            .alongWith(getCollectCommand(intake, feeder)))
        .andThen(getToggleIntakeCommand(intake));
  }

  public static final Command getTrackNoteSpikeCommand(
      Drive drive, Intake intake, Feeder feeder, Vision noteVision) {
    return (DriveCommands.moveTowardsTarget(
                drive, noteVision, FieldConstants.startingLineX + 0.5, VisionMode.Notes)
            .alongWith(getCollectCommand(intake, feeder)))
        .andThen(getToggleIntakeCommand(intake));
  }

  public static final Command getTrackSpeakerFarCommand(
      Drive drive, Hood hood, Shooter shooter, Vision aprilTagVision) {
    return DriveCommands.moveTowardsTarget(drive, aprilTagVision, 3.75, VisionMode.AprilTags);
  }

  public static final Command getTrackSpeakerCloseCommand(
      Drive drive, Hood hood, Shooter shooter, Vision aprilTagVision) {
    return DriveCommands.moveTowardsTarget(
        drive, aprilTagVision, FieldConstants.startingLineX - 0.25, VisionMode.AprilTags);
  }

  public static final Command getCollectCommand(Intake intake, Feeder feeder) {
    return Commands.sequence(
        intake.deployIntake(), Commands.parallel(intake.runVoltage(), feeder.intake()));
  }

  public static final Command getToggleIntakeCommand(Intake intake) {
    return intake.toggleIntake();
  }

  public static final Command getAccelerateShooterCommand(
      Drive drive, Hood hood, Shooter shooter, Vision aprilTagVision) {
    return shooter
        .runVelocity()
        .alongWith(
            hood.setPosition(
                () -> Optional.of(aprilTagVision.getRobotPose().get().getTranslation()),
                drive::getFieldRelativeVelocity));
  }

  public static final Command getShootCommand(Feeder feeder) { // this will change to be automatic once the shooter is up to speed and will be along with any command that needs to shoot a note
    return feeder.shoot();
  }

  public static final Command getAmpCommand(Shooter shooter, Amp amp) {
    return shooter.runAmp().alongWith(amp.setAmp());
  }
}
