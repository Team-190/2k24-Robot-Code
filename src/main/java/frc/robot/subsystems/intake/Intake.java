package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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
      case ROBOT_2K24_C:
      case ROBOT_2K24_P:
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

  private void stopRollers() {
    io.setRollersVoltage(0.0);
  }

  private void setIntakePosition(Value position) {
    io.setIntakePosition(position);
  }

  private void toggleIntakePosition() {
    io.toggleIntakePosition();
  }

  public Command runVoltage() {
    return runEnd(() -> io.setRollersVoltage(rollersVoltage.get()), () -> stopRollers());
  }

  public Command deployIntake() {
    return runOnce(() -> setIntakePosition(Value.kForward));
  }

  public Command retractIntake() {
    return runOnce(() -> setIntakePosition(Value.kReverse));
  }

  public Command toggleIntake() {
    return runOnce(() -> toggleIntakePosition());
  }
}
