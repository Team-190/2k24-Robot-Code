package frc.robot.subsystems.accelerator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Accelerator extends SubsystemBase {

  private final AcceleratorIO io;
  private final AcceleratorIOInputsAutoLogged inputs = new AcceleratorIOInputsAutoLogged();
  private static final LoggedTunableNumber accelerateVoltage =
      new LoggedTunableNumber("Accelerator/Shoot Voltage");

  public Accelerator(AcceleratorIO io) {
    this.io = io;
    accelerateVoltage.initDefault(12.0);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Accelerator", inputs);
  }

  private void stop() {
    io.setVoltage(0.0);
  }

  public Command runAccelerator() {
    return runEnd(
        () -> {
          io.setVoltage(accelerateVoltage.get());
        },
        () -> stop());
  }
}
