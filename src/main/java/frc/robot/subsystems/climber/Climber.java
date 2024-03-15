package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
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

  private static final LoggedTunableNumber OFFSET = new LoggedTunableNumber("Climber/Offset");

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private final ProfiledPIDController leftProfiledFeedback;
  private final ProfiledPIDController rightProfiledFeedback;

  static {
    LOW_POSITION.initDefault(7);
    HIGH_POSITION.initDefault(9);
    OFFSET.initDefault(0);
    switch (Constants.ROBOT) {
      case SNAPBACK:
        KP.initDefault(5);
        KD.initDefault(0.0);
        MAX_VELOCITY.initDefault(30);
        MAX_ACCELERATION.initDefault(20);
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
        KP.initDefault(1.0);
        KD.initDefault(0.0);
        MAX_VELOCITY.initDefault(1.0);
        MAX_ACCELERATION.initDefault(1.0);
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

    leftProfiledFeedback.setTolerance(9);
    rightProfiledFeedback.setTolerance(9);
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
      if (inputs.lockedPosition) {
        io.setLeftVoltage((leftProfiledFeedback.calculate(inputs.leftPositionMeters)));
        io.setRightVoltage((rightProfiledFeedback.calculate(inputs.rightPositionMeters)));
      }
    }

    if (DriverStation.isDisabled()) {
      io.setLock(true);
      leftProfiledFeedback.reset(inputs.leftPositionMeters, 0.0);
      rightProfiledFeedback.reset(inputs.rightPositionMeters, 0.0);
    }

    Logger.recordOutput("Climber/Left/Goal", leftProfiledFeedback.getGoal().position);
    Logger.recordOutput("Climber/Left/Setpoint", leftProfiledFeedback.getSetpoint().position);
    Logger.recordOutput("Climber/Right/Goal", rightProfiledFeedback.getGoal().position);
    Logger.recordOutput("Climber/Right/Setpoint", rightProfiledFeedback.getSetpoint().position);
  }

  private void setLeftPosition(double leftPositionMeters) {
    leftProfiledFeedback.setGoal(leftPositionMeters + OFFSET.get());
  }

  private void setRightPosition(double rightPositionMeters) {
    rightProfiledFeedback.setGoal(rightPositionMeters + OFFSET.get());
  }

  public double getLeftPositionMeters() {
    return inputs.leftPositionMeters;
  }

  public double getRightPositionMeters() {
    return inputs.rightPositionMeters;
  }

  public Command preClimbCenter() {
    return Commands.runOnce(() -> io.setLock(false))
        .andThen(Commands.waitSeconds(0.25))
        .andThen(
            Commands.runOnce(
                () -> {
                  setLeftPosition(LOW_POSITION.get());
                  setRightPosition(LOW_POSITION.get());
                }))
        .andThen(
            () -> {
              if (leftProfiledFeedback.atGoal() && rightProfiledFeedback.atGoal()) {
                io.setLock(true);
              }
            });
  }

  public Command preClimbSide() {
    return Commands.runOnce(() -> io.setLock(false))
        .andThen(Commands.waitSeconds(0.25))
        .andThen(
            Commands.runOnce(
                () -> {
                  setLeftPosition(HIGH_POSITION.get());
                  setRightPosition(HIGH_POSITION.get());
                }))
        .andThen(
            () -> {
              if (leftProfiledFeedback.atGoal() && rightProfiledFeedback.atGoal()) {
                io.setLock(true);
              }
            });
  }

  public Command climb() {
    return Commands.runOnce(() -> io.setLock(false))
        .andThen(Commands.waitSeconds(0.25))
        .andThen(
            Commands.runOnce(
                () -> {
                  setLeftPosition(STOWED_POSITION.get());
                  setRightPosition(STOWED_POSITION.get());
                }))
        .andThen(
            () -> {
              if (leftProfiledFeedback.atGoal() && rightProfiledFeedback.atGoal()) {
                io.setLock(true);
              }
            });
  }

  public Command stop() {
    return Commands.run(
        () -> {
          io.setLeftVoltage(0.0);
          io.setRightVoltage(0.0);
          io.setLock(true);
        });
  }

  public Command incrementClimber() {
    return Commands.runOnce(() -> OFFSET.initDefault(OFFSET.get() + Units.inchesToMeters(1)));
  }
}
