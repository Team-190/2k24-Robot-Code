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

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Mode;
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.accelerator.Accelerator;
import frc.robot.subsystems.accelerator.AcceleratorIO;
import frc.robot.subsystems.accelerator.AcceleratorIOSim;
import frc.robot.subsystems.accelerator.AcceleratorIOTalonFX;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.amp.AmpIO;
import frc.robot.subsystems.amp.AmpIOSim;
import frc.robot.subsystems.amp.AmpIOTalonFX;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerIO;
import frc.robot.subsystems.kicker.KickerIOSim;
import frc.robot.subsystems.kicker.KickerIOTalonFX;
import frc.robot.subsystems.serializer.Serializer;
import frc.robot.subsystems.serializer.SerializerIO;
import frc.robot.subsystems.serializer.SerializerIOSim;
import frc.robot.subsystems.serializer.SerializerIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.subsystems.vision.VisionMode;
import frc.robot.util.AutoBuilderNameChanger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private Drive drive;
  private Shooter shooter;
  private Hood hood;
  private Intake intake;
  private Serializer serializer;
  private Kicker kicker;
  private Accelerator accelerator;
  private Amp amp;
  private Climber climber;
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
          hood = new Hood(new HoodIOTalonFX());
          intake = new Intake(new IntakeIOTalonFX());
          serializer = new Serializer(new SerializerIOTalonFX());
          kicker = new Kicker(new KickerIOTalonFX());
          accelerator = new Accelerator(new AcceleratorIOTalonFX());
          amp = new Amp(new AmpIOTalonFX());
          climber = new Climber(new ClimberIOTalonFX());
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
          hood = new Hood(new HoodIOSim());
          intake = new Intake(new IntakeIOSim());
          serializer = new Serializer(new SerializerIOSim());
          kicker = new Kicker(new KickerIOSim());
          accelerator = new Accelerator(new AcceleratorIOSim());
          amp = new Amp(new AmpIOSim());
          climber = new Climber(new ClimberIOSim());
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
    if (hood == null) {
      hood = new Hood(new HoodIO() {});
    }
    if (intake == null) {
      intake = new Intake(new IntakeIO() {});
    }
    if (serializer == null) {
      serializer = new Serializer(new SerializerIO() {});
    }
    if (kicker == null) {
      kicker = new Kicker(new KickerIO() {});
    }
    if (accelerator == null) {
      accelerator = new Accelerator(new AcceleratorIO() {});
    }
    if (amp == null) {
      amp = new Amp(new AmpIO() {});
    }
    if (climber == null) {
      climber = new Climber(new ClimberIO() {});
    }
    if (aprilTagVision == null) {
      aprilTagVision = new Vision("AprilTagVision", new VisionIO() {});
    }
    if (noteVision == null) {
      noteVision = new Vision("NoteVision", new VisionIO() {});
    }

    // Set up suppliers
    aprilTagVision.setDrivePoseSupplier(drive::getPose);
    noteVision.setDrivePoseSupplier(drive::getPose);

    // Pathplanner commands
    NamedCommands.registerCommand(
        "Delay", Commands.waitSeconds(SmartDashboard.getNumber("Auto Start Shooting Delay", 0.0)));
    NamedCommands.registerCommand(
        "Shoot", CompositeCommands.getShootCommand(serializer, kicker).withTimeout(0.5));
    NamedCommands.registerCommand(
        "Feed", CompositeCommands.getFeedCommand(intake, serializer, kicker));
    NamedCommands.registerCommand(
        "Track Note Center",
        CompositeCommands.getTrackNoteCenterCommand(drive, intake, serializer, noteVision));
    NamedCommands.registerCommand(
        "Track Note Spike",
        CompositeCommands.getTrackNoteSpikeCommand(drive, intake, serializer, noteVision));
    NamedCommands.registerCommand(
        "Track Speaker Far",
        CompositeCommands.getTrackSpeakerFarCommand(drive, hood, shooter, aprilTagVision));
    NamedCommands.registerCommand(
        "Track Speaker Close",
        CompositeCommands.getTrackSpeakerCloseCommand(drive, hood, shooter, aprilTagVision));
    NamedCommands.registerCommand(
        "Shoot On The Move",
        CompositeCommands.shootOnTheMove(drive, serializer, kicker, aprilTagVision));

    // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser =
        new LoggedDashboardChooser<>(
            "Auto Choices", AutoBuilderNameChanger.buildNameChangedAutoChooser());

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
    controller.y().whileTrue(CompositeCommands.getAmpCommand(shooter, hood, amp));
    controller.leftTrigger().whileTrue(CompositeCommands.getCollectCommand(intake, serializer));
    controller.leftBumper().onTrue(CompositeCommands.getToggleIntakeCommand(intake));
    controller
        .a()
        .toggleOnTrue(
            CompositeCommands.getAccelerateShooterCommand(
                drive, hood, shooter, accelerator, aprilTagVision));
    controller
        .rightTrigger()
        .and(shooter::isShooting)
        .whileTrue(CompositeCommands.getShootCommand(serializer, kicker));
    controller.back().onTrue(climber.preClimbCenter());
    controller.start().onTrue(climber.preClimbSide());
    controller.povUp().onTrue(climber.climb());
  }

  public Command getAutonomousCommand() {
    return Constants.TUNING_MODE
        ? autoChooser.get()
        : autoChooser
            .get()
            .alongWith(
                CompositeCommands.getAccelerateShooterCommand(
                    drive, hood, shooter, accelerator, aprilTagVision));
  }
}
