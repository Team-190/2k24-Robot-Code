package frc.robot.subsystems.snapback.climber;

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

  public Command setLeftVoltage(double volts) {
    return runEnd(() -> io.setLeftVoltage(volts), () -> io.setLeftVoltage(0.0));
  }

  public Command setRightVoltage(double volts) {
    return runEnd(() -> io.setRightVoltage(volts), () -> io.setRightVoltage(0.0));
  }

  public Command raiseClimber() {
    return Commands.parallel(
        setLeftVoltage(12.0)
            .until(() -> inputs.leftPositionMeters >= ClimberConstants.CLIMBER_MAX_HEIGHT_METERS),
        setRightVoltage(12.0)
            .until(() -> inputs.rightPositionMeters >= ClimberConstants.CLIMBER_MAX_HEIGHT_METERS));
  }

  public Command lowerClimber() {
    return Commands.parallel(
        setLeftVoltage(-12.0)
            .until(() -> inputs.leftPositionMeters <= ClimberConstants.CLIMBER_MIN_HEIGHT_METERS),
        setRightVoltage(-12.0)
            .until(() -> inputs.rightPositionMeters <= ClimberConstants.CLIMBER_MIN_HEIGHT_METERS));
  }

  public Command stop() {
    return Commands.run(
        () -> {
          io.setLeftVoltage(0.0);
          io.setRightVoltage(0.0);
        });
  }

  public Command climb(BooleanSupplier startClimb) {
    return Commands.sequence(
        raiseClimber(), stop(), Commands.waitUntil(startClimb), lowerClimber(), stop());
  }
}
