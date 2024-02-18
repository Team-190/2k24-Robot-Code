package frc.robot.subsystems.amp;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Amp extends SubsystemBase {
  private final AmpIO io;
  private final AmpIOInputsAutoLogged inputs = new AmpIOInputsAutoLogged();

  static {
    switch (Constants.ROBOT) {
      case ROBOT_2K24_C:
      case ROBOT_2K24_P:
        break;
      case ROBOT_2K24_TEST:
        break;
      case ROBOT_SIM:
        break;
      default:
        break;
    }
  }

  public Amp(AmpIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Amp", inputs);

    Logger.recordOutput("Amp/position", inputs.position);
  }

  private void setPosition(DoubleSolenoid.Value position) {
    io.setPosition(position);
  }

  public Command setAmp() {
    return startEnd(
        () -> setPosition(DoubleSolenoid.Value.kForward),
        () -> setPosition(DoubleSolenoid.Value.kReverse));
  }
}
