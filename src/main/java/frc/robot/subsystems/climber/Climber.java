package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private static final LoggedTunableNumber KP = new LoggedTunableNumber("Climber/Kp");
  private static final LoggedTunableNumber KD = new LoggedTunableNumber("Climber/Kd");
  private static final LoggedTunableNumber MAX_VELOCITY =
      new LoggedTunableNumber("Climber/Max Velocity");
  private static final LoggedTunableNumber MAX_ACCELERATION =
      new LoggedTunableNumber("Climber/Max Acceleration");

  private static final LoggedTunableNumber STOWED_POSITION =
      new LoggedTunableNumber("Climber/Stowed Position");

  private static final LoggedTunableNumber HIGH_POSITION =
      new LoggedTunableNumber("Climber/High Position");

  private static final LoggedTunableNumber LOW_POSITION =
      new LoggedTunableNumber("Climber/Low Position");

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private final ProfiledPIDController leftProfiledFeedback;
  private final ProfiledPIDController rightProfiledFeedback;

  static {
    switch (Constants.ROBOT) {
      case ROBOT_2K24_C:
      case ROBOT_2K24_P:
        KP.initDefault(0.0);
        KD.initDefault(0.0);
        MAX_VELOCITY.initDefault(0.0);
        MAX_ACCELERATION.initDefault(0.0);
        STOWED_POSITION.initDefault(0.0);
        break;
      case ROBOT_2K24_TEST:
        KP.initDefault(0.0);
        KD.initDefault(0.0);
        MAX_VELOCITY.initDefault(0.0);
        MAX_ACCELERATION.initDefault(0.0);
        STOWED_POSITION.initDefault(0.0);
        break;
      case ROBOT_SIM:
        KP.initDefault(0.0);
        KD.initDefault(0.0);
        MAX_VELOCITY.initDefault(0.0);
        MAX_ACCELERATION.initDefault(0.0);
        STOWED_POSITION.initDefault(0.0);
        break;
      default:
        break;
    }
  }

  public Climber(ClimberIO io) {
    this.io = io;
    leftProfiledFeedback =
        new ProfiledPIDController(
            KP.get(), 0.0, KD.get(), new Constraints(MAX_VELOCITY.get(), MAX_ACCELERATION.get()));
    rightProfiledFeedback =
        new ProfiledPIDController(
            KP.get(), 0.0, KD.get(), new Constraints(MAX_VELOCITY.get(), MAX_ACCELERATION.get()));
    setDefaultCommand(run(() -> stop()));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    if (KP.hasChanged(hashCode())) {
      leftProfiledFeedback.setP(KP.get());
      rightProfiledFeedback.setP(KP.get());
    }
    if (KD.hasChanged(hashCode())) {
      leftProfiledFeedback.setD(KD.get());
      rightProfiledFeedback.setD(KD.get());
    }
    if (MAX_VELOCITY.hasChanged(hashCode()) || MAX_ACCELERATION.hasChanged(hashCode())) {
      leftProfiledFeedback.setConstraints(
          new Constraints(MAX_VELOCITY.get(), MAX_ACCELERATION.get()));
      rightProfiledFeedback.setConstraints(
          new Constraints(MAX_VELOCITY.get(), MAX_ACCELERATION.get()));
    }

    if (DriverStation.isEnabled()) {
      io.setLeftVoltage((leftProfiledFeedback.calculate(inputs.leftPositionMeters)));
      io.setRightVoltage((rightProfiledFeedback.calculate(inputs.rightPositionMeters)));
    }

    if (DriverStation.isDisabled()) {
      leftProfiledFeedback.reset(inputs.leftPositionMeters, 0.0);
      rightProfiledFeedback.reset(inputs.rightPositionMeters, 0.0);
    }

    Logger.recordOutput("Climber/Left/Goal", leftProfiledFeedback.getGoal().position);
    Logger.recordOutput("Climber/Left/Setpoint", leftProfiledFeedback.getSetpoint().position);
    Logger.recordOutput("Climber/Right/Goal", rightProfiledFeedback.getGoal().position);
    Logger.recordOutput("Climber/Right/Setpoint", rightProfiledFeedback.getSetpoint().position);
  }

  private void setLeftPosition(double leftPositionMeters) {
    leftProfiledFeedback.setGoal(leftPositionMeters);
  }

  private void setRightPosition(double rightPositionMeters) {
    rightProfiledFeedback.setGoal(rightPositionMeters);
  }

  public double getLeftPositionMeters() {
    return inputs.leftPositionMeters;
  }

  public double getRightPositionMeters() {
    return inputs.rightPositionMeters;
  }

  public Command preClimbCenter() {
    return Commands.runOnce(
        () -> {
          setLeftPosition(LOW_POSITION.get());
          setRightPosition(LOW_POSITION.get());
        });
  }

  public Command preClimbSide() {
    return Commands.runOnce(
        () -> {
          setLeftPosition(HIGH_POSITION.get());
          setRightPosition(HIGH_POSITION.get());
        });
  }

  public Command climb() {
    return Commands.runOnce(
        () -> {
          setLeftPosition(STOWED_POSITION.get());
          setRightPosition(STOWED_POSITION.get());
        });
  }

  private void stop() {
    io.setLeftVoltage(0.0);
    io.setRightVoltage(0.0);
  }
}
