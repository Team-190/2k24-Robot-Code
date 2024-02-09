package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {

  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
  private static final LoggedTunableNumber shootUpperVoltage =
      new LoggedTunableNumber("Feeder/shootUpperVoltage");
  private static final LoggedTunableNumber shootLowerVoltage =
      new LoggedTunableNumber("Feeder/shootLowerVoltage");
  private static final LoggedTunableNumber intakeVoltage =
      new LoggedTunableNumber("Feeder/Intake voltage");

  private static final DigitalInput sensor = new DigitalInput(0);

  public Feeder(FeederIO io) {
    this.io = io;
    shootUpperVoltage.initDefault(9.0);
    shootLowerVoltage.initDefault(9.0);
    intakeVoltage.initDefault(12.0);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
    Logger.recordOutput("sensor has note", sensor.get());
  }

  private void stop() {
    io.setUpperVoltage(0.0);
    io.setLowerVoltage(0.0);
  }

  public Command shoot() {
    return runEnd(
        () -> {
          io.setUpperVoltage(shootUpperVoltage.get());
          io.setLowerVoltage(shootLowerVoltage.get());
        },
        () -> stop());
  }

  public Command intake() {
    return runEnd(() -> io.setLowerVoltage(intakeVoltage.get()), () -> stop())
        .until(() -> sensor.get());
  }
}
