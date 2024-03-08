package frc.robot.commands;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.ShotCalculator;
import frc.robot.subsystems.accelerator.Accelerator;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.serializer.Serializer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionMode;
import java.util.Optional;

public class CompositeCommands {
  public static final Command getCollectCommand(Intake intake, Serializer serializer) {
    return Commands.sequence(
        intake.deployIntake(), Commands.parallel(intake.runVoltage(), serializer.intake()));
  }

  public static final Command getRetractCommand(Intake intake) {
    return intake.retractIntake();
  }

  public static final Command getToggleIntakeCommand(Intake intake) {
    return intake.toggleIntake();
  }

  public static final Command getPosePrepShooterCommand(
      Drive drive, Hood hood, Shooter shooter, Accelerator accelerator, Vision aprilTagVision) {
    return shooter
        .runPoseDistance(
            () -> Optional.of(aprilTagVision.getRobotPose().get().getTranslation()),
            drive::getFieldRelativeVelocity)
        .alongWith(
            hood.setPosePosition(
                () -> Optional.of(aprilTagVision.getRobotPose().get().getTranslation()),
                drive::getFieldRelativeVelocity))
        .alongWith(accelerator.runAccelerator());
  }

  public static final Command getAnglePrepShooterCommand(
      Drive drive, Hood hood, Shooter shooter, Accelerator accelerator, Vision aprilTagVision) {
    return shooter
        .runAngleDistance()
        .alongWith(hood.setAnglePosition())
        .alongWith(accelerator.runAccelerator());
  }

  public static final Command getShootCommand(Serializer serializer, Kicker kicker) {
    return serializer.shoot().alongWith(kicker.shoot());
  }

  public static final Command getFeedCommand(Intake intake, Serializer serializer, Kicker kicker) {
    return Commands.sequence(
        intake.deployIntake(),
        Commands.parallel(intake.runVoltage(), serializer.intake(), kicker.shoot()));
  }

  public static final Command getAmpCommand(Shooter shooter, Hood hood, Amp amp) {
    return shooter.runAmp().alongWith(hood.setAmp()).alongWith(amp.setAmp());
  }

  public static final Command getTrackNoteCenterCommand(
      Drive drive, Intake intake, Serializer serializer, Vision noteVision) {
    return (DriveCommands.moveTowardsTarget(
                drive, noteVision, FieldConstants.fieldLength / 2.0, VisionMode.Notes)
            .raceWith(getCollectCommand(intake, serializer)))
        .andThen(getToggleIntakeCommand(intake));
  }

  public static final Command getTrackNoteSpikeCommand(
      Drive drive, Intake intake, Serializer serializer, Vision noteVision) {
    return (DriveCommands.moveTowardsTarget(
                drive, noteVision, FieldConstants.startingLineX + 0.5, VisionMode.Notes)
            .raceWith(getCollectCommand(intake, serializer)))
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

  public static final Command shootOnTheMove(
      Drive drive, Serializer serializer, Kicker kicker, Vision vision) {
    return Commands.run(
            () -> {
              if (vision.getRobotPose().isPresent()) {
                PPHolonomicDriveController.setRotationTargetOverride(
                    () ->
                        Optional.of(
                            ShotCalculator.poseCalculation(
                                    vision.getRobotPose().get().getTranslation(),
                                    drive.getFieldRelativeVelocity())
                                .robotAngle()));
              }
            })
        .alongWith(getShootCommand(serializer, kicker));
  }
}
