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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Mode;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.GrantFieldRelativeDrive;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.util.FrskyController;

public class RobotContainer {
  // Subsystems
  private Drive drive;
  // private Shooter shooter;
  // private Hood hood;
  // private Intake intake;
  // private Serializer serializer;
  // private Kicker kicker;
  // private Accelerator accelerator;
  // private Amp amp;

  // Controller
  private final FrskyController driver = new FrskyController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
      }

      // Configure the button bindings
      configureButtonBindings();
    }
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        new GrantFieldRelativeDrive(
            drive,
            () -> driver.getLeftStickY(),
            () -> driver.getRightStickX(),
            () -> driver.getRightStickY(),
            () -> -driver.getLeftStickX()));

    driver.topRightSwitch.onTrue(DriveCommands.resetHeading(drive));
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
