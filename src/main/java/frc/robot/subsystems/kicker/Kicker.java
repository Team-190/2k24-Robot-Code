package frc.robot.subsystems.kicker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {

  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();
  private static final LoggedTunableNumber shootVoltage =
      new LoggedTunableNumber("Kicker/Shoot Voltage");
  private static final LoggedTunableNumber intakeVoltage =
      new LoggedTunableNumber("Kicker/Intake voltage");

  public Kicker(KickerIO io) {
    this.io = io;
    shootVoltage.initDefault(12.0);
    intakeVoltage.initDefault(12.0);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);
  }

  private void stop() {
    io.setVoltage(0.0);
  }

  public Command shoot() {
    return runEnd(
        () -> {
          io.setVoltage(shootVoltage.get());
        },
        () -> stop());
  }

  public Command run() {
    return runEnd(() -> io.setVoltage(intakeVoltage.get()), () -> stop());
  }
}
