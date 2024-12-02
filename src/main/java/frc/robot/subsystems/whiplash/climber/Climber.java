package frc.robot.subsystems.whiplash.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs;

  public Climber(ClimberIO io) {
    this.io = io;
    inputs = new ClimberIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command setClimberVoltage(double volts) {
    return runEnd(() -> io.setClimberVoltage(volts), () -> io.setClimberVoltage(0.0));
  }

  public Command raiseClimber() {
    return Commands.parallel(
        setClimberVoltage(12.0)
            .until(() -> inputs.climberPositionMeters >= ClimberConstants.CLIMBER_MAX_HEIGHT_METERS));
  }

  public Command lowerClimber() {
    return Commands.parallel(
        setClimberVoltage(-12.0)
            .until(() -> inputs.climberPositionMeters <= ClimberConstants.CLIMBER_MIN_HEIGHT_METERS));
  }

  public Command stop() {
    return Commands.run(
        () -> {
          io.setClimberVoltage(0.0);
        });
  }

  public Command climb(BooleanSupplier startClimb) {
    return Commands.sequence(
        raiseClimber(), stop(), Commands.waitUntil(startClimb), lowerClimber(), stop());
  }
}
