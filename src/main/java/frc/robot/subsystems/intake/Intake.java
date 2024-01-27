package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private static final LoggedTunableNumber voltage = new LoggedTunableNumber("Intake/voltage");

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
    return startEnd(
        () -> {
          io.setVoltage(voltage.get());
        },
        () -> {
          stop();
        });
  }
}
