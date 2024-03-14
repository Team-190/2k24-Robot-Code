package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private static final LoggedTunableNumber rollersVoltage =
      new LoggedTunableNumber("Intake/Voltage");

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  static {
    switch (Constants.ROBOT) {
      case SNAPBACK:
        rollersVoltage.initDefault(7);
        break;
      case ROBOT_2K24_TEST:
        rollersVoltage.initDefault(7);
        break;
      case ROBOT_SIM:
        rollersVoltage.initDefault(7);
        break;
      default:
        break;
    }
  }

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public boolean isDeployed() {
    return inputs.leftPosition && inputs.rightPosition;
  }

  private void stopRollers() {
    io.setRollersVoltage(0.0);
  }

  private void setIntakePosition(boolean position) {
    io.setIntakePosition(position);
  }

  private void toggleIntakePosition() {
    io.toggleIntakePosition();
  }

  public Command runVoltage() {
    return runEnd(() -> io.setRollersVoltage(rollersVoltage.get()), () -> stopRollers());
  }

  public Command outtake() {
    return runEnd(() -> io.setRollersVoltage(-12), () -> stopRollers());
  }

  public Command deployIntake() {
    return runOnce(() -> setIntakePosition(true));
  }

  public Command retractIntake() {
    return runOnce(() -> setIntakePosition(false));
  }

  public Command toggleIntake() {
    return runOnce(() -> toggleIntakePosition());
  }
}
