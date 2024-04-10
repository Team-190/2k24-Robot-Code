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
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
import frc.robot.subsystems.leds.Leds;
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
import frc.robot.util.SnapbackMechanism3d;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

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
  private Leds leds;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Tunable Numbers
  private final LoggedDashboardNumber autoDelay = new LoggedDashboardNumber("Auto Delay");

  // Note tracking
  private static boolean isNoteTracking = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    leds = Leds.getInstance();
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.ROBOT) {
        case SNAPBACK:
          // Snapback, instantiate hardware IO implementations
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
          // amp = new Amp(new AmpIOTalonFX());
          climber = new Climber(new ClimberIOTalonFX());
          aprilTagVision =
              new Vision("AprilTagVision", new VisionIOLimelight(VisionMode.AprilTags));
          noteVision = new Vision("NoteVision", new VisionIOLimelight(VisionMode.Notes));
          break;
        case ROBOT_2K24_TEST:
          // Test robot, instantiate hardware IO implementations
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(0),
                  new ModuleIOTalonFX(1),
                  new ModuleIOTalonFX(2),
                  new ModuleIOTalonFX(3));
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
    leds.setNoteSupplier(serializer::hasNote);
    leds.setPrepSupplier(shooter::isShooting);
    leds.setShootSupplier(kicker::isShooting);

    // Pathplanner commands
    NamedCommands.registerCommand("Deploy", intake.deployIntake());
    NamedCommands.registerCommand(
        "Shoot",
        (Commands.waitUntil(() -> ShotCalculator.shooterReady(hood, shooter))
                .andThen(
                    CompositeCommands.getShootCommand(intake, serializer, kicker)
                        .withTimeout(0.25)))
            .withTimeout(2));
    NamedCommands.registerCommand(
        "Feed", CompositeCommands.getFeedCommand(intake, serializer, kicker));
    NamedCommands.registerCommand(
        "Track Note Center",
        CompositeCommands.getTrackNoteCenterCommand(
            drive, intake, serializer, noteVision, aprilTagVision));
    NamedCommands.registerCommand(
        "Track Note Spike",
        CompositeCommands.getTrackNoteSpikeCommand(
            drive, intake, serializer, noteVision, aprilTagVision));
    NamedCommands.registerCommand(
        "Track Note Spike Medium",
        CompositeCommands.getTrackNoteSpikeCommand(
            drive, intake, serializer, noteVision, aprilTagVision, 1.5));
    NamedCommands.registerCommand(
        "Track Note Spike Slow",
        CompositeCommands.getTrackNoteSpikeCommand(
            drive, intake, serializer, noteVision, aprilTagVision, 0.5));
    NamedCommands.registerCommand(
        "Track Speaker Far",
        CompositeCommands.getTrackSpeakerFarCommand(drive, hood, shooter, aprilTagVision));
    NamedCommands.registerCommand(
        "Track Speaker Close",
        CompositeCommands.getTrackSpeakerCloseCommand(drive, hood, shooter, aprilTagVision));
    NamedCommands.registerCommand(
        "Aim", CompositeCommands.getAimSpeakerCommand(drive, aprilTagVision));
    NamedCommands.registerCommand(
        "Set Rightmost",
        DriverStation.getAlliance().isPresent()
            ? (DriverStation.getAlliance().get().equals(Alliance.Red)
                ? (noteVision.setPipeline(2))
                : (noteVision.setPipeline(1)))
            : (noteVision.setPipeline(0)));
    NamedCommands.registerCommand(
        "Set Leftmost",
        DriverStation.getAlliance().isPresent()
            ? (DriverStation.getAlliance().get().equals(Alliance.Red)
                ? (noteVision.setPipeline(1))
                : (noteVision.setPipeline(3)))
            : (noteVision.setPipeline(0)));

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

      autoChooser.addOption("Hood Test", hood.setAmp());
      autoChooser.addOption("Amp Test", amp.deployAmp());
      autoChooser.addOption("Intake Test", intake.deployIntake());
    }

    // Configure the button bindings
    configureButtonBindings();

    // Configure shuffleboard
    Shuffleboard.getTab("Autonomous")
        .add("Autonomous Mode", autoChooser.getSendableChooser())
        .withPosition(0, 0)
        .withSize(2, 2);
    Shuffleboard.getTab("Teleoperated")
        .addNumber("Hood Offset", hood::getOffset)
        .withPosition(0, 0)
        .withSize(1, 1);
    Shuffleboard.getTab("Teleoperated")
        .addNumber("Flywheel Offset", shooter::getFlywheelOffset)
        .withPosition(0, 1)
        .withSize(1, 1);
    Shuffleboard.getTab("Teleoperated")
        .addNumber("Spin Offset", shooter::getSpinOffset)
        .withPosition(0, 2)
        .withSize(1, 1);
    Shuffleboard.getTab("Teleoperated")
        .add("Shooter View", CameraServer.addSwitchedCamera("limelight-shooter").getSource())
        .withPosition(1, 0)
        .withSize(5, 5)
        .withWidget("Camera Stream");
    Shuffleboard.getTab("Teleoperated")
        .add("Intake View", CameraServer.addSwitchedCamera("limelight-intake").getSource())
        .withPosition(6, 0)
        .withSize(5, 5)
        .withWidget("Camera Stream");
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            aprilTagVision,
            noteVision,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            driver.rightBumper(),
            driver
                .leftBumper()
                .and(() -> isNoteTracking))); // change to driver.leftBumper().and(() ->
    // isNoteTracking) for note
    // tracking
    driver.y().onTrue(DriveCommands.resetHeading(drive));
    driver
        .rightTrigger()
        .whileTrue(CompositeCommands.getSourceFeedCommand(shooter, hood, amp, accelerator, kicker))
        .onFalse(amp.retractAmp());
    driver.leftTrigger().whileTrue(CompositeCommands.getOuttakeCommand(intake, serializer, kicker));
    driver
        .leftBumper()
        .whileTrue(
            CompositeCommands.getCollectCommand(intake, serializer, noteVision, aprilTagVision))
        .onFalse(intake.retractIntake());
    driver
        .rightBumper()
        .whileTrue(
            CompositeCommands.getPosePrepShooterCommand(
                drive, hood, shooter, accelerator, aprilTagVision));
    driver
        .rightBumper()
        .and(() -> ShotCalculator.shooterReady(hood, shooter))
        .whileTrue(
            Commands.waitSeconds(0.25)
                .andThen(CompositeCommands.getShootCommand(intake, serializer, kicker))
                .withTimeout(0.5));
    driver.b().whileTrue(CompositeCommands.getShootCommand(intake, serializer, kicker));
    driver.a().whileTrue(intake.singleActuation());

    operator
        .rightBumper()
        .whileTrue(CompositeCommands.getAmpFeedCommand(shooter, hood, amp, accelerator, kicker));
    operator.y().whileTrue(shooter.increaseVelocity());
    operator.a().whileTrue(shooter.decreaseVelocity());
    operator
        .rightTrigger()
        .whileTrue(CompositeCommands.getAmpCommand(shooter, hood, amp, accelerator, kicker))
        .onFalse(amp.retractAmp());
    operator.povUp().onTrue(climber.preClimb());
    operator.povDown().onTrue(climber.climbAutomatic());
    operator.back().onTrue(climber.zero());
    operator.leftBumper().onTrue(Commands.runOnce(() -> isNoteTracking = !isNoteTracking));
  }

  public void updateSnapbackMechanism3d() {
    Logger.recordOutput(
        "Mechanism3d",
        SnapbackMechanism3d.getPoses(
            intake.isDeployed(),
            hood.getPosition(),
            climber.getLeftPositionMeters(),
            climber.getRightPositionMeters()));
  }

  public void updatePoseCalculation() {
    if (aprilTagVision.getRobotPose().isPresent()) {
      ShotCalculator.poseCalculation(
          aprilTagVision.getRobotPose().get().getTranslation(), drive.getFieldRelativeVelocity());
      Logger.recordOutput("Shooter Ready", ShotCalculator.shooterReady(hood, shooter));
    }
  }

  public void resetVisionPipelines() {
    noteVision.setPipeline(0);
  }

  public LoggedDashboardChooser<Command> getAutoChooser() {
    return autoChooser;
  }

  public Command getAutonomousCommand() {
    return (Commands.waitSeconds(autoDelay.get()).andThen(autoChooser.get().asProxy()))
        .alongWith(
            CompositeCommands.getPosePrepShooterCommand(
                drive, hood, shooter, accelerator, aprilTagVision));
  }
}
