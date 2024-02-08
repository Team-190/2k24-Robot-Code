package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private static final LoggedTunableNumber voltage = new LoggedTunableNumber("Intake/voltage");

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  static {
    switch (Constants.ROBOT) {
      case ROBOT_2K24_C:
      case ROBOT_2K24_P:
        voltage.initDefault(3);
        break;
      case ROBOT_2K24_TEST:
        voltage.initDefault(3);
        break;
      case ROBOT_SIM:
        voltage.initDefault(3);
        break;
      default:
        break;
    }
  }

  public Intake(IntakeIO io) {
    this.io = io;
    voltage.initDefault(0.0);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  private void stop() {
    io.setVoltage(0.0);
  }

  public Command runVoltage() {
    return runEnd(() -> io.setVoltage(voltage.get()), () -> stop());
  }
}
