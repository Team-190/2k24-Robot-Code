package frc.robot.subsystems.serializer;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Serializer extends SubsystemBase {

  private final SerializerIO io;
  private final SerializerIOInputsAutoLogged inputs = new SerializerIOInputsAutoLogged();
  private static final LoggedTunableNumber shootVoltage =
      new LoggedTunableNumber("Serializer/Shoot Voltage");
  private static final LoggedTunableNumber intakeVoltage =
      new LoggedTunableNumber("Serializer/Intake Voltage");

  private static final DigitalInput sensor = new DigitalInput(0);

  public Serializer(SerializerIO io) {
    this.io = io;
    shootVoltage.initDefault(12.0);
    intakeVoltage.initDefault(12.0);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Serializer", inputs);
    Logger.recordOutput("Note?", sensor.get());
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

  public Command intake() {
    return runEnd(() -> io.setVoltage(intakeVoltage.get()), () -> stop());
  }
}
