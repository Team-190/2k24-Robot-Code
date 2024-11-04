package frc.robot;

import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.drive.DriveConstants;
import frc.robot.subsystems.shared.drive.GyroIO;
import frc.robot.subsystems.shared.drive.GyroIOPigeon2;
import frc.robot.subsystems.shared.drive.ModuleIO;
import frc.robot.subsystems.shared.drive.ModuleIOSim;
import frc.robot.subsystems.shared.drive.ModuleIOTalonFX;
import frc.robot.subsystems.shared.vision.Vision;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private Drive drive;
  private Vision vision;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);

  // Dashboard Inputs
  private LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.ROBOT) {
        case SNAPBACK:
        case WHIPLASH:
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DriveConstants.FRONT_LEFT),
                  new ModuleIOTalonFX(DriveConstants.FRONT_RIGHT),
                  new ModuleIOTalonFX(DriveConstants.BACK_LEFT),
                  new ModuleIOTalonFX(DriveConstants.BACK_RIGHT));
          vision = new Vision();
          break;
        case ROBOT_SIM:
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(DriveConstants.FRONT_LEFT),
                  new ModuleIOSim(DriveConstants.FRONT_RIGHT),
                  new ModuleIOSim(DriveConstants.BACK_LEFT),
                  new ModuleIOSim(DriveConstants.BACK_RIGHT));
          vision = new Vision();
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

    // Configure auto choices.
    autoChooser = new LoggedDashboardChooser<>("Auto Routines");
    autoChooser.addDefaultOption("None", Commands.none());
    if (Constants.TUNING_MODE) {
      autoChooser.addOption(
          "Drive Quasistatic Forward",
          DriveCommands.runSysIdQuasistatic(drive, Direction.kForward));
      autoChooser.addOption(
          "Drive Quasistatic Reverse",
          DriveCommands.runSysIdQuasistatic(drive, Direction.kReverse));
      autoChooser.addOption(
          "Drive Dynamic Forward", DriveCommands.runSysIdDynamic(drive, Direction.kForward));
      autoChooser.addOption(
          "Drive Dynamic Reverse", DriveCommands.runSysIdDynamic(drive, Direction.kReverse));
    }

    Shuffleboard.getTab("Autonomous")
        .add("Autonomous Mode", autoChooser.getSendableChooser())
        .withPosition(0, 0)
        .withSize(2, 2);
    Shuffleboard.getTab("Teleoperated")
        .addBoolean("Note?", () -> false)
        .withPosition(0, 0)
        .withSize(8, 5);

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            driver.rightBumper(),
            driver.axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.05),
            driver.b()));
    driver.y().onTrue(CompositeCommands.resetHeading());
  }

  public void robotPeriodic() {
    RobotState.periodic(
        drive.getRawGyroRotation(),
        NetworkTablesJNI.now(),
        drive.getYawVelocity(),
        drive.getFieldRelativeVelocity(),
        drive.getModulePositions(),
        vision.getCameras());
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
