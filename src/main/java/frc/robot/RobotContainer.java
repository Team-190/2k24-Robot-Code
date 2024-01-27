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
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Mode;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import org.littletonrobotics.junction.Logger;
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
  private Feeder feeder;
  private Intake intake;
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
          // drive =
          //     new Drive(
          //         new GyroIOPigeon2(),
          //         new ModuleIOTalonFX(0),
          //         new ModuleIOTalonFX(1),
          //         new ModuleIOTalonFX(2),
          //         new ModuleIOTalonFX(3));
          intake = new Intake(new IntakeIOTalonFX());
          feeder = new Feeder(new FeederIOTalonFX());
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
          intake = new Intake(new IntakeIOSim());
          feeder = new Feeder(new FeederIOSim());
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
    if (intake == null) {
      intake = new Intake(new IntakeIO() {});
    }
    if (feeder == null) {
      feeder = new Feeder(new FeederIO() {});
    }

    // Set up autos
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId
    if (Constants.TUNING_MODE) {
      var driveSysId =
          new SysIdRoutine(
              new SysIdRoutine.Config(
                  null, null, null, (state) -> Logger.recordOutput("SysIdState", state.toString())),
              new SysIdRoutine.Mechanism(
                  (volts) -> drive.runCharacterizationVolts(volts.in(Units.Volts)), null, drive));
      autoChooser.addOption(
          "Drive SysId (Quasistatic Forward)", driveSysId.quasistatic(Direction.kForward));
      autoChooser.addOption(
          "Drive SysId (Quasistatic Reverse)", driveSysId.quasistatic(Direction.kReverse));
      autoChooser.addOption(
          "Drive SysId (Dynamic Forward)", driveSysId.dynamic(Direction.kForward));
      autoChooser.addOption(
          "Drive SysId (Dynamic Reverse)", driveSysId.dynamic(Direction.kReverse));
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
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    controller.x().onTrue(DriveCommands.XLock(drive));
    controller.b().onTrue(DriveCommands.resetHeading(drive));
    // controller.a().onTrue(testSubsystem.runBoth());
    controller.leftTrigger().whileTrue(intake.runVoltage());
    controller.rightTrigger().whileTrue(feeder.runVoltage());
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
