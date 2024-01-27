package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {

  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
  private static final LoggedTunableNumber voltage = new LoggedTunableNumber("Feeder/voltage");

  public Feeder(FeederIO io) {
    this.io = io;
    voltage.initDefault(0.0);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
  }

  private void stop() {
    io.setVoltage(0.0);
  }

  public Command runVoltage() {
    return startEnd(
        () -> {
          io.setVoltage(voltage.get());
        },
        () -> {
          stop();
        });
  }
}
