package frc.robot.subsystems.amp;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Amp extends SubsystemBase {
  private static final LoggedTunableNumber KP = new LoggedTunableNumber("Amp/Kp");
  private static final LoggedTunableNumber KD = new LoggedTunableNumber("Amp/Kd");
  private static final LoggedTunableNumber MAX_VELOCITY =
      new LoggedTunableNumber("Amp/Max Velocity");
  private static final LoggedTunableNumber MAX_ACCELERATION =
      new LoggedTunableNumber("Amp/Max Acceleration");

  private static final LoggedTunableNumber STOWED_POSITION =
      new LoggedTunableNumber("Amp/Stowed Position");

  private static final LoggedTunableNumber AMP_POSITION =
      new LoggedTunableNumber("Amp/Amp Position");

  private final AmpIO io;
  private final AmpIOInputsAutoLogged inputs = new AmpIOInputsAutoLogged();

  private final ProfiledPIDController profiledFeedback;

  static {
    switch (Constants.ROBOT) {
      case SNAPBACK:
        KP.initDefault(0.0);
        KD.initDefault(0.0);
        MAX_VELOCITY.initDefault(0.0);
        MAX_ACCELERATION.initDefault(0.0);
        STOWED_POSITION.initDefault(0.0);
        AMP_POSITION.initDefault(0.0);
        break;
      case ROBOT_2K24_TEST:
        KP.initDefault(0.0);
        KD.initDefault(0.0);
        MAX_VELOCITY.initDefault(0.0);
        MAX_ACCELERATION.initDefault(0.0);
        STOWED_POSITION.initDefault(0.0);
        AMP_POSITION.initDefault(0.0);

        break;
      case ROBOT_SIM:
        KP.initDefault(1.0);
        KD.initDefault(0.05);
        MAX_VELOCITY.initDefault(500);
        MAX_ACCELERATION.initDefault(500);
        STOWED_POSITION.initDefault(0.0);
        AMP_POSITION.initDefault(Units.degreesToRadians(135.0));

        break;
      default:
        break;
    }
  }

  public Amp(AmpIO io) {
    this.io = io;
    profiledFeedback =
        new ProfiledPIDController(
            KP.get(), 0.0, KD.get(), new Constraints(MAX_VELOCITY.get(), MAX_ACCELERATION.get()));
    setDefaultCommand(
        run(
            () -> {
              setPosition(STOWED_POSITION.get());
            }));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Amp", inputs);

    if (KP.hasChanged(hashCode())) {
      profiledFeedback.setP(KP.get());
    }
    if (KD.hasChanged(hashCode())) {
      profiledFeedback.setD(KD.get());
    }
    if (MAX_VELOCITY.hasChanged(hashCode()) || MAX_ACCELERATION.hasChanged(hashCode())) {
      profiledFeedback.setConstraints(new Constraints(MAX_VELOCITY.get(), MAX_ACCELERATION.get()));
    }

    if (DriverStation.isEnabled()) {
      io.setVoltage(profiledFeedback.calculate(inputs.position.getRadians()));
    }

    if (DriverStation.isDisabled()) {
      profiledFeedback.reset(inputs.position.getRadians(), 0);
    }

    Logger.recordOutput("Amp/Goal", profiledFeedback.getGoal().position);
    Logger.recordOutput("Amp/Setpoint", profiledFeedback.getSetpoint().position);
  }

  private void setPosition(double positionRad) {
    profiledFeedback.setGoal(positionRad);
  }

  public Rotation2d getPosition() {
    return inputs.position;
  }

  public Command setAmp() {
    return runEnd(() -> setPosition(AMP_POSITION.get()), () -> setPosition(STOWED_POSITION.get()));
  }
}
