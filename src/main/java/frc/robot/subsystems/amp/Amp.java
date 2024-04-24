package frc.robot.subsystems.amp;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Amp extends SubsystemBase {

  private final AmpIO io;
  private final AmpIOInputsAutoLogged inputs = new AmpIOInputsAutoLogged();

  public Amp(AmpIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Amp", inputs);

    Logger.recordOutput("Amp/Position", inputs.position);
  }

  private void setPosition(boolean isDeployed) {
    io.setPosition(isDeployed);
  }

  public Command deployAmp() {
    return runOnce(() -> setPosition(true));
  }

  public Command retractAmp() {
    return runOnce(() -> setPosition(false));
  }
}
