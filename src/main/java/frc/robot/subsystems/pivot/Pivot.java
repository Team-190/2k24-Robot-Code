package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private static final LoggedTunableNumber KP = new LoggedTunableNumber("Pivot/Kp");
  private static final LoggedTunableNumber KD = new LoggedTunableNumber("Pivot/Kd");
  private static final LoggedTunableNumber MAX_VELOCITY =
      new LoggedTunableNumber("Pivot/Max Velocity");
  private static final LoggedTunableNumber MAX_ACCELERATION =
      new LoggedTunableNumber("Pivot/Max Acceleration");

  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private final ProfiledPIDController profiledFeedback;

  static {
    switch (Constants.ROBOT) {
      case ROBOT_2K24_C:
      case ROBOT_2K24_P:
        KP.initDefault(0.0);
        KD.initDefault(0.0);
        MAX_VELOCITY.initDefault(0.0);
        MAX_ACCELERATION.initDefault(0.0);
        break;
      case ROBOT_2K24_TEST:
        KP.initDefault(0.0);
        KD.initDefault(0.0);
        MAX_VELOCITY.initDefault(0.0);
        MAX_ACCELERATION.initDefault(0.0);
        break;
      case ROBOT_SIM:
        KP.initDefault(0.0);
        KD.initDefault(0.0);
        MAX_VELOCITY.initDefault(0.0);
        MAX_ACCELERATION.initDefault(0.0);
        break;
      default:
        break;
    }
  }

  public Pivot(PivotIO io) {
    this.io = io;

    profiledFeedback =
        new ProfiledPIDController(
            KP.get(), 0.0, KD.get(), new Constraints(MAX_VELOCITY.get(), MAX_ACCELERATION.get()));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    if (KP.hasChanged(hashCode())) {
      profiledFeedback.setP(KP.get());
    }
    if (KD.hasChanged(hashCode())) {
      profiledFeedback.setD(KD.get());
    }
    if (MAX_VELOCITY.hasChanged(hashCode()) || MAX_ACCELERATION.hasChanged(hashCode())) {
      profiledFeedback.setConstraints(new Constraints(MAX_VELOCITY.get(), MAX_ACCELERATION.get()));
    }

    io.setVoltage(profiledFeedback.calculate(inputs.position.getRadians()));
  }

  public void setPosition(double positionRad) {
    profiledFeedback.setGoal(positionRad);
  }

  public Command pivot180() {
    return startEnd(() -> setPosition(Math.PI), () -> setPosition(0.0));
  }

  public Command pivot90() {
    return startEnd(() -> setPosition(Math.PI / 2.0), () -> setPosition(0.0));
  }
}
