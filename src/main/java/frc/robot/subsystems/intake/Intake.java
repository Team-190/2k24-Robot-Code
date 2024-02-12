package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private static final LoggedTunableNumber rollersVoltage = new LoggedTunableNumber("Rollers/voltage");

  private static final LoggedTunableNumber INTAKE_KP = new LoggedTunableNumber("Intake/kP");
  private static final LoggedTunableNumber INTAKE_KD = new LoggedTunableNumber("Intake/kD");

  private static final LoggedTunableNumber INTAKE_MAX_VELOCITY =
      new LoggedTunableNumber("Intake/Max Velocity");
  private static final LoggedTunableNumber INTAKE_MAX_ACCELERATION =
      new LoggedTunableNumber("Intake/Max Acceleration");

  private static final LoggedTunableNumber INTAKE_DEPLOYED_POSITION =
      new LoggedTunableNumber("Intake/Deployed Position");
  private static final LoggedTunableNumber INTAKE_STOWED_POSITION =
      new LoggedTunableNumber("Intake/Stowed Position");

  private final ProfiledPIDController profiledFeedback;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  static {
    switch (Constants.ROBOT) {
      case ROBOT_2K24_C:
      case ROBOT_2K24_P:
        rollersVoltage.initDefault(7);
        INTAKE_KP.initDefault(0);
        INTAKE_KD.initDefault(0);
        INTAKE_MAX_VELOCITY.initDefault(0);
        INTAKE_MAX_ACCELERATION.initDefault(0);
        INTAKE_DEPLOYED_POSITION.initDefault(0);
        break;
      case ROBOT_2K24_TEST:
        rollersVoltage.initDefault(7);
        INTAKE_KP.initDefault(0);
        INTAKE_KD.initDefault(0);
        INTAKE_MAX_VELOCITY.initDefault(0);
        INTAKE_MAX_ACCELERATION.initDefault(0);
        INTAKE_DEPLOYED_POSITION.initDefault(0);
        break;
      case ROBOT_SIM:
        rollersVoltage.initDefault(7);
        INTAKE_KP.initDefault(0);
        INTAKE_KD.initDefault(0);
        INTAKE_MAX_VELOCITY.initDefault(0);
        INTAKE_MAX_ACCELERATION.initDefault(0);
        INTAKE_DEPLOYED_POSITION.initDefault(0);
        break;
      default:
        break;
    }
  }

  public Intake(IntakeIO io) {
    this.io = io;
    profiledFeedback =
        new ProfiledPIDController(
            INTAKE_KP.get(),
            0.0,
            INTAKE_KD.get(),
            new Constraints(INTAKE_MAX_VELOCITY.get(), INTAKE_MAX_ACCELERATION.get()));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    if (INTAKE_KP.hasChanged(hashCode())) {
      profiledFeedback.setP(INTAKE_KP.get());
    }
    if (INTAKE_KD.hasChanged(hashCode())) {
      profiledFeedback.setP(INTAKE_KD.get());
    }
    if (INTAKE_MAX_VELOCITY.hasChanged(hashCode())
        || INTAKE_MAX_ACCELERATION.hasChanged(hashCode())) {
      profiledFeedback.setConstraints(
          new Constraints(INTAKE_MAX_VELOCITY.get(), INTAKE_MAX_ACCELERATION.get()));
    }

    if (DriverStation.isEnabled()) {
      io.setIntakeVoltage(profiledFeedback.calculate(inputs.intakePositionRad.getRadians()));
    }

    if (DriverStation.isDisabled()) {
      profiledFeedback.reset(inputs.intakePositionRad.getRadians(), 0);
    }

    Logger.recordOutput("Pivot/goal", profiledFeedback.getGoal().position);
    Logger.recordOutput("Pivot/setpoint", profiledFeedback.getSetpoint().position);
  }

  private void stopRollers() {
    io.setRollersVoltage(0.0);
  }

  public void setPosition(double positionRad) {
    profiledFeedback.setGoal(positionRad);
  }

  public Command runVoltage() {
    return runEnd(() -> io.setRollersVoltage(rollersVoltage.get()), () -> stopRollers());
  }

  public Command collect() {
    return startEnd(
        () -> {
          setPosition(INTAKE_DEPLOYED_POSITION.get());
          runVoltage();
        },
        () -> setPosition(INTAKE_STOWED_POSITION.get()));
  }
}
