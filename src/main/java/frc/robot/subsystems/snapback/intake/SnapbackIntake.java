package frc.robot.subsystems.snapback.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class SnapbackIntake extends SubsystemBase {
  private final SnapbackIntakeIO io;
  private final SnapbackIntakeIOInputsAutoLogged inputs;

  private boolean isIntaking;

  public SnapbackIntake(SnapbackIntakeIO io) {
    this.io = io;
    inputs = new SnapbackIntakeIOInputsAutoLogged();

    isIntaking = false;
  }

  @Override
  public void periodic() {
    switch (Constants.ROBOT) {
      case SNAPBACK:
      case SNAPBACK_SIM:
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        break;
      default:
        break;
    }
  }

  public boolean sensorTriggered() {
    return inputs.sensorValue;
  }

  public boolean isIntaking() {
    return isIntaking;
  }

  private Command runAllMotors(double voltage) {
    return Commands.parallel(
        Commands.run(() -> io.setIntakeVoltage(voltage)),
        Commands.run(() -> io.setSerializerVoltage(voltage)),
        Commands.run(() -> io.setKickerVoltage(voltage / 2)));
  }

  public Command intake() {
    return runEnd(
        () -> {
          isIntaking = true;
          runOnce(() -> io.setActuatorValue(Value.kForward));
          runAllMotors(12).until(() -> sensorTriggered());
        },
        () -> {
          runOnce(() -> io.setActuatorValue(Value.kReverse));
          runAllMotors(0);
          isIntaking = false;
        });
  }

  public Command outtake() {
    return runEnd(() -> runAllMotors(-12), () -> runAllMotors(0));
  }

  public Command shoot() {
    return runEnd(() -> io.setKickerVoltage(12.0), () -> io.setKickerVoltage(0.0));
  }
}
