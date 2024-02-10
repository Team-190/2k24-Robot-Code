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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Mode;
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.subsystems.vision.VisionMode;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private Drive drive;
  private Shooter shooter;
  private Feeder feeder;
  private Intake intake;
  private Pivot pivot;
  private Vision aprilTagVision;
  private Vision noteVision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.ROBOT) {
        case ROBOT_2K24_C:
        case ROBOT_2K24_P:
        case ROBOT_2K24_TEST:
          // Real robot, instantiate hardware IO implementations
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(0),
                  new ModuleIOTalonFX(1),
                  new ModuleIOTalonFX(2),
                  new ModuleIOTalonFX(3));
          shooter = new Shooter(new ShooterIOTalonFX());
          intake = new Intake(new IntakeIOTalonFX());
          feeder = new Feeder(new FeederIOTalonFX());
          // pivot = new Pivot(new PivotIOTalonFX());
          aprilTagVision =
              new Vision("AprilTagVision", new VisionIOLimelight(VisionMode.AprilTags));
          noteVision = new Vision("NoteVision", new VisionIOLimelight(VisionMode.Notes));
          break;

        case ROBOT_SIM:
          // Sim robot, instantiate physics sim IO implementations
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
          shooter = new Shooter(new ShooterIOSim());
          intake = new Intake(new IntakeIOSim());
          feeder = new Feeder(new FeederIOSim());
          pivot = new Pivot(new PivotIOSim());
          aprilTagVision =
              new Vision("AprilTagVision", new VisionIOSim(VisionMode.AprilTags, drive::getPose));
          noteVision = new Vision("NoteVision", new VisionIOSim(VisionMode.Notes, drive::getPose));
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
    if (shooter == null) {
      shooter = new Shooter(new ShooterIO() {});
    }
    if (intake == null) {
      intake = new Intake(new IntakeIO() {});
    }
    if (feeder == null) {
      feeder = new Feeder(new FeederIO() {});
    }
    if (pivot == null) {
      pivot = new Pivot(new PivotIO() {});
    }
    if (aprilTagVision == null) {
      aprilTagVision = new Vision("AprilTagVision", new VisionIO() {});
    }
    if (noteVision == null) {
      noteVision = new Vision("NoteVision", new VisionIO() {});
    }

    // Set up subsystems
    aprilTagVision.setDrivePoseSupplier(drive::getPose);
    noteVision.setDrivePoseSupplier(drive::getPose);

    // Set up autos
    NamedCommands.registerCommand(
        "Track Note Center",
        CompositeCommands.getTrackNoteCenterCommand(drive, intake, feeder, noteVision));
    NamedCommands.registerCommand(
        "Track Note Spike",
        CompositeCommands.getTrackNoteSpikeCommand(drive, intake, feeder, noteVision));
    NamedCommands.registerCommand(
        "Track Speaker Far", CompositeCommands.getTrackSpeakerFarCommand(drive, aprilTagVision));
    NamedCommands.registerCommand(
        "Track Speaker Close",
        CompositeCommands.getTrackSpeakerCloseCommand(drive, aprilTagVision));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId
    if (Constants.TUNING_MODE) {
      autoChooser.addOption(
          "Drive SysId (Quasistatic Forward)",
          DriveCommands.runSysIdQuasistatic(drive, Direction.kForward));
      autoChooser.addOption(
          "Drive SysId (Quasistatic Reverse)",
          DriveCommands.runSysIdQuasistatic(drive, Direction.kReverse));
      autoChooser.addOption(
          "Drive SysId (Dynamic Forward)",
          DriveCommands.runSysIdDynamic(drive, Direction.kForward));
      autoChooser.addOption(
          "Drive SysId (Dynamic Reverse)",
          DriveCommands.runSysIdDynamic(drive, Direction.kReverse));
      autoChooser.addOption("Shooter SysId", shooter.runSysId());
    }

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            aprilTagVision,
            noteVision,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX(),
            controller.leftBumper(),
            controller.rightBumper()));
    controller.x().onTrue(DriveCommands.XLock(drive));
    controller.b().onTrue(DriveCommands.resetHeading(drive));
    controller.leftTrigger().whileTrue(CompositeCommands.getIntakeCommand(intake, feeder));
    controller.a().toggleOnTrue(CompositeCommands.getAccelerateShooterCommand(shooter));
    controller
        .rightTrigger()
        .and(shooter::isShooting)
        .whileTrue(CompositeCommands.getShootCommand(feeder));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
