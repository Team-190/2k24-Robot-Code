package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Mode;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.drive.Drive;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drive.module.ModuleConstants;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.drive.module.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
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
                  new ModuleIOTalonFX(ModuleConstants.FRONT_LEFT),
                  new ModuleIOTalonFX(ModuleConstants.FRONT_RIGHT),
                  new ModuleIOTalonFX(ModuleConstants.REAR_LEFT),
                  new ModuleIOTalonFX(ModuleConstants.REAR_RIGHT));
          vision = new Vision();
          break;
        case ROBOT_SIM:
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
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
    autoChooser.addDefaultOption("None", AutoRoutines.none());
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
            () -> Math.copySign(Math.pow(-driver.getLeftY(), 2), -driver.getLeftY()),
            () -> Math.copySign(Math.pow(-driver.getLeftX(), 2), -driver.getLeftX()),
            () -> Math.copySign(Math.pow(driver.getRightX(), 2), driver.getRightX())));
    driver.y().onTrue(CompositeCommands.resetHeading(drive));
  }

  public void robotPeriodic() {
    RobotState.periodic(
        drive.getRotation(),
        drive.getYawVelocity(),
        drive.getFieldRelativeVelocity(),
        drive.getModulePositions(),
        vision.getCameras(),
        vision.getValidTarget(),
        vision.getPrimaryVisionPoses(),
        vision.getSecondaryVisionPoses(),
        vision.getFrameTimestamps());
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
