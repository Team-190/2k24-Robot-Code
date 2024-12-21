// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Mode;
import frc.robot.Constants.RobotType;
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.drive.DriveConstants;
import frc.robot.subsystems.shared.drive.GyroIO;
import frc.robot.subsystems.shared.drive.GyroIOPigeon2;
import frc.robot.subsystems.shared.drive.ModuleIO;
import frc.robot.subsystems.shared.drive.ModuleIOSim;
import frc.robot.subsystems.shared.drive.ModuleIOTalonFX;
import frc.robot.subsystems.shared.vision.CameraConstants;
import frc.robot.subsystems.shared.vision.Vision;
import frc.robot.subsystems.snapback.hood.SnapbackHood;
import frc.robot.subsystems.snapback.hood.SnapbackHoodIO;
import frc.robot.subsystems.snapback.hood.SnapbackHoodIOSim;
import frc.robot.subsystems.snapback.hood.SnapbackHoodIOTalonFX;
import frc.robot.subsystems.snapback.intake.SnapbackIntake;
import frc.robot.subsystems.snapback.intake.SnapbackIntakeIO;
import frc.robot.subsystems.snapback.intake.SnapbackIntakeIOSim;
import frc.robot.subsystems.snapback.intake.SnapbackIntakeIOTalonFX;
import frc.robot.subsystems.snapback.shooter.SnapbackShooter;
import frc.robot.subsystems.snapback.shooter.SnapbackShooterIO;
import frc.robot.subsystems.snapback.shooter.SnapbackShooterIOSim;
import frc.robot.subsystems.snapback.shooter.SnapbackShooterIOTalonFX;
import frc.robot.subsystems.whiplash.arm.WhiplashArm;
import frc.robot.subsystems.whiplash.arm.WhiplashArmIO;
import frc.robot.subsystems.whiplash.arm.WhiplashArmIOSim;
import frc.robot.subsystems.whiplash.arm.WhiplashArmIOTalonFX;
import frc.robot.subsystems.whiplash.intake.WhiplashIntake;
import frc.robot.subsystems.whiplash.intake.WhiplashIntakeIO;
import frc.robot.subsystems.whiplash.intake.WhiplashIntakeIOSim;
import frc.robot.subsystems.whiplash.intake.WhiplashIntakeIOTalonFX;
import frc.robot.subsystems.whiplash.shooter.WhiplashShooter;
import frc.robot.subsystems.whiplash.shooter.WhiplashShooterIO;
import frc.robot.subsystems.whiplash.shooter.WhiplashShooterIOSim;
import frc.robot.subsystems.whiplash.shooter.WhiplashShooterIOTalonFX;
import frc.robot.util.LTNUpdater;

public class RobotContainer {
  // Subsystems
  private Drive drive;
  private Vision vision;

  private SnapbackIntake snapbackIntake;
  private SnapbackHood snapbackHood;
  private SnapbackShooter snapbackShooter;

  private WhiplashIntake whiplashIntake;
  private WhiplashArm whiplashArm;
  private WhiplashShooter whiplashShooter;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.ROBOT) {
        case SNAPBACK:
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DriveConstants.FRONT_LEFT),
                  new ModuleIOTalonFX(DriveConstants.FRONT_RIGHT),
                  new ModuleIOTalonFX(DriveConstants.BACK_LEFT),
                  new ModuleIOTalonFX(DriveConstants.BACK_RIGHT));
          vision =
              new Vision(
                  CameraConstants.RobotCameras.LEFT_CAMERA,
                  CameraConstants.RobotCameras.RIGHT_CAMERA);
          snapbackIntake = new SnapbackIntake(new SnapbackIntakeIOTalonFX());
          snapbackHood = new SnapbackHood(new SnapbackHoodIOTalonFX());
          snapbackShooter = new SnapbackShooter(new SnapbackShooterIOTalonFX());
          break;
        case WHIPLASH:
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DriveConstants.FRONT_LEFT),
                  new ModuleIOTalonFX(DriveConstants.FRONT_RIGHT),
                  new ModuleIOTalonFX(DriveConstants.BACK_LEFT),
                  new ModuleIOTalonFX(DriveConstants.BACK_RIGHT));
          vision =
              new Vision(
                  CameraConstants.RobotCameras.LEFT_CAMERA,
                  CameraConstants.RobotCameras.RIGHT_CAMERA);
          whiplashIntake = new WhiplashIntake(new WhiplashIntakeIOTalonFX());
          whiplashArm = new WhiplashArm(new WhiplashArmIOTalonFX());
          whiplashShooter = new WhiplashShooter(new WhiplashShooterIOTalonFX());
          break;
        case SNAPBACK_SIM:
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOSim(DriveConstants.FRONT_LEFT),
                  new ModuleIOSim(DriveConstants.FRONT_RIGHT),
                  new ModuleIOSim(DriveConstants.BACK_LEFT),
                  new ModuleIOSim(DriveConstants.BACK_RIGHT));
          vision =
              new Vision(
                  CameraConstants.RobotCameras.LEFT_CAMERA,
                  CameraConstants.RobotCameras.RIGHT_CAMERA);
          snapbackIntake = new SnapbackIntake(new SnapbackIntakeIOSim());
          snapbackHood = new SnapbackHood(new SnapbackHoodIOSim());
          snapbackShooter = new SnapbackShooter(new SnapbackShooterIOSim());
          break;
        case WHIPLASH_SIM:
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOSim(DriveConstants.FRONT_LEFT),
                  new ModuleIOSim(DriveConstants.FRONT_RIGHT),
                  new ModuleIOSim(DriveConstants.BACK_LEFT),
                  new ModuleIOSim(DriveConstants.BACK_RIGHT));
          vision =
              new Vision(
                  CameraConstants.RobotCameras.LEFT_CAMERA,
                  CameraConstants.RobotCameras.RIGHT_CAMERA);
          whiplashIntake = new WhiplashIntake(new WhiplashIntakeIOSim());
          whiplashArm = new WhiplashArm(new WhiplashArmIOSim());
          whiplashShooter = new WhiplashShooter(new WhiplashShooterIOSim());
          break;
      }
    }

    // Instantiate missing subsystems
    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }
    if (vision == null) {
      vision =
          new Vision(
              CameraConstants.ReplayCameras.LEFT_CAMERA,
              CameraConstants.ReplayCameras.RIGHT_CAMERA);
    }

    switch (Constants.ROBOT) {
      case SNAPBACK:
      case SNAPBACK_SIM:
        if (snapbackIntake == null) {
          snapbackIntake = new SnapbackIntake(new SnapbackIntakeIO() {});
        }
        if (snapbackHood == null) {
          snapbackHood = new SnapbackHood(new SnapbackHoodIO() {});
        }
        if (snapbackShooter == null) {
          snapbackShooter = new SnapbackShooter(new SnapbackShooterIO() {});
        }
        snapbackConfigureButtonBindings();
        break;
      case WHIPLASH:
      case WHIPLASH_SIM:
        if (whiplashIntake == null) {
          whiplashIntake = new WhiplashIntake(new WhiplashIntakeIO() {});
        }
        if (whiplashArm == null) {
          whiplashArm = new WhiplashArm(new WhiplashArmIO() {});
        }
        if (whiplashShooter == null) {
          whiplashShooter = new WhiplashShooter(new WhiplashShooterIO() {});
        }
        whiplashConfigureButtonBindings();
        break;
    }
  }

  private void whiplashConfigureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            driver.rightBumper(),
            driver.axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.05),
            driver.b()));
    driver.y().onTrue(CompositeCommands.resetHeading(drive));
    driver.leftBumper().whileTrue(CompositeCommands.collect(whiplashIntake, whiplashArm));
    driver.leftTrigger().whileTrue(CompositeCommands.eject(whiplashIntake, whiplashArm));
    driver
        .rightBumper()
        .whileTrue(
            CompositeCommands.shootSpeaker(
                drive, whiplashIntake, whiplashArm, whiplashShooter, driver.getHID()));
    driver
        .x()
        .whileTrue(CompositeCommands.shootSubwoofer(whiplashIntake, whiplashArm, whiplashShooter));
    driver
        .axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.95)
        .whileTrue(CompositeCommands.shootAmp(whiplashIntake, whiplashArm, whiplashShooter));
    driver
        .axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.05)
        .whileTrue(whiplashArm.ampAngle());
    driver
        .povUp()
        .onTrue(
            Commands.runOnce(
                    () ->
                        RobotState.resetRobotPose(
                            new Pose2d(
                                FieldConstants.Subwoofer.centerFace.getTranslation(),
                                RobotState.getRobotPose().getRotation())))
                .ignoringDisable(true));
    driver.a().whileTrue(CompositeCommands.shootFeed(whiplashIntake, whiplashArm, whiplashShooter));
    operator.leftBumper().whileTrue(CompositeCommands.collect(whiplashIntake, whiplashArm));
  }

  private void snapbackConfigureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            driver.rightBumper(),
            driver.axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.05),
            driver.b()));
    driver.y().onTrue(CompositeCommands.resetHeading(drive));
    driver.leftBumper().whileTrue(CompositeCommands.collect(snapbackIntake));
    driver.leftTrigger().whileTrue(CompositeCommands.eject(snapbackIntake));
    driver
        .rightBumper()
        .whileTrue(
            CompositeCommands.shootSpeaker(
                drive, snapbackIntake, snapbackHood, snapbackShooter, driver.getHID()));
    driver
        .povUp()
        .onTrue(
            Commands.runOnce(
                    () ->
                        RobotState.resetRobotPose(
                            new Pose2d(
                                FieldConstants.Subwoofer.centerFace.getTranslation(),
                                RobotState.getRobotPose().getRotation())))
                .ignoringDisable(true));
    driver
        .a()
        .whileTrue(CompositeCommands.shootFeed(snapbackIntake, snapbackHood, snapbackShooter));
    operator.leftBumper().whileTrue(CompositeCommands.collect(snapbackIntake));
  }

  public void robotPeriodic() {
    RobotState.periodic(
        drive.getRawGyroRotation(),
        NetworkTablesJNI.now(),
        drive.getYawVelocity(),
        drive.getFieldRelativeVelocity(),
        drive.getModulePositions(),
        vision.getCameras(),
        false,
        false,
        false);
    if (Constants.ROBOT == RobotType.WHIPLASH || Constants.ROBOT == RobotType.WHIPLASH_SIM) {
      LTNUpdater.updateWhiplash(drive, whiplashArm, whiplashIntake, whiplashShooter);
    }
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
