package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class Climber extends SubsystemBase {
  private static final LoggedTunableNumber KP = new LoggedTunableNumber("Climber/Kp");
  private static final LoggedTunableNumber KD = new LoggedTunableNumber("Climber/Kd");
  private static final LoggedTunableNumber MAX_VELOCITY =
      new LoggedTunableNumber("Climber/MaxVelocity");
  private static final LoggedTunableNumber MAX_ACCELERATION =
      new LoggedTunableNumber("Climber/MaxAcceleration");

  private static final LoggedTunableNumber STOWED_POSITION =
      new LoggedTunableNumber("Climber/Stowed Position");

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
    setDefaultCommand(
        run(() -> stop()));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    if (KP.hasChanged(hashCode())) {
      leftProfiledFeedback.setP(KP.get());
            rightProfiledFeedback.setP(KP.get());

    }
    if (KD.hasChanged(hashCode())) {
      leftProfiledFeedback.setP(KD.get());
            rightProfiledFeedback.setP(KD.get());
    }
    if (MAX_VELOCITY.hasChanged(hashCode()) || MAX_ACCELERATION.hasChanged(hashCode())) {
      leftProfiledFeedback.setConstraints(new Constraints(MAX_VELOCITY.get(), MAX_ACCELERATION.get()));
            rightProfiledFeedback.setConstraints(new Constraints(MAX_VELOCITY.get(), MAX_ACCELERATION.get()));

    }

    if (DriverStation.isEnabled()) {
      io.setLeftVoltage((leftProfiledFeedback.calculate(inputs.leftPositionMeters)));
            io.setRightVoltage((rightProfiledFeedback.calculate(inputs.rightPositionMeters)));

    }

    if (DriverStation.isDisabled()) {
      leftProfiledFeedback.reset(inputs.leftPositionMeters, 0.0);
      rightProfiledFeedback.reset(inputs.rightPositionMeters, 0.0);
    }

    Logger.recordOutput("Climber/Left/goal", leftProfiledFeedback.getGoal().position);
    Logger.recordOutput("Climber/Left/setpoint", leftProfiledFeedback.getSetpoint().position);
    Logger.recordOutput("Climber/Right/goal", rightProfiledFeedback.getGoal().position);
    Logger.recordOutput("Climber/Right/setpoint", rightProfiledFeedback.getSetpoint().position);
  }

  public void setLeftPosition(double leftPositionMeters) {
    leftProfiledFeedback.setGoal(leftPositionMeters);
  }

  public void setRightPosition(double rightPositionMeters) {
    rightProfiledFeedback.setGoal(rightPositionMeters);
  }

  private void stop() {
    io.setLeftVoltage(0.0);
    io.setRightVoltage(0.0);
  }
}
