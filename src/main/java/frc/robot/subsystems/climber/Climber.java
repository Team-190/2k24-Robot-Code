package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
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

  private static final LoggedTunableNumber CLIMB_POSITION =
      new LoggedTunableNumber("Climber/Climb Position");

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private final ProfiledPIDController leftProfiledFeedback;
  private final ProfiledPIDController rightProfiledFeedback;

  static {
    CLIMB_POSITION.initDefault(11);
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

    leftProfiledFeedback.setTolerance(3);
    rightProfiledFeedback.setTolerance(3);
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

  private Command setLeftPosition(double leftPositionMeters) {
    return Commands.run(
            () ->
                io.setLeftVoltage(
                    leftProfiledFeedback.calculate(inputs.leftPositionMeters, leftPositionMeters)))
        .until(() -> leftProfiledFeedback.atGoal());
  }

  private Command setRightPosition(double rightPositionMeters) {
    return Commands.run(
            () ->
                io.setRightVoltage(
                    rightProfiledFeedback.calculate(
                        inputs.rightPositionMeters, rightPositionMeters)))
        .until(() -> rightProfiledFeedback.atGoal());
  }

  public double getLeftPositionMeters() {
    return inputs.leftPositionMeters;
  }

  public double getRightPositionMeters() {
    return inputs.rightPositionMeters;
  }

  public Command preClimb() {
    return Commands.runOnce(() -> io.setLock(false))
        .andThen(Commands.waitSeconds(0.25))
        .andThen(
            setLeftPosition(CLIMB_POSITION.get()).alongWith(setRightPosition(CLIMB_POSITION.get())))
        .andThen(() -> io.setLock(true));
  }

  public Command climb(DoubleSupplier speed, double deadband) {
    return Commands.runOnce(() -> io.setLock(false))
        .andThen(Commands.waitSeconds(0.25))
        .andThen(
            Commands.either(
                Commands.runEnd(
                    () -> {
                      io.setLeftVoltage(
                          -speed.getAsDouble() * RobotController.getBatteryVoltage() * 0.25);
                      io.setRightVoltage(
                          -speed.getAsDouble() * RobotController.getBatteryVoltage() * 0.25);
                    },
                    () -> {
                      io.setLeftVoltage(0);
                      io.setRightVoltage(0);
                      io.setLock(true);
                    }),
                Commands.run(
                    () -> {
                      io.setLeftVoltage(0);
                      io.setRightVoltage(0);
                      io.setLock(true);
                    }),
                () -> Math.abs(speed.getAsDouble()) > deadband));
  }

  public Command stop() {
    return Commands.run(
        () -> {
          io.setLeftVoltage(0.0);
          io.setRightVoltage(0.0);
          io.setLock(true);
        });
  }
}
