package frc.robot.subsystems.hood;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ShotCalculator;
import frc.robot.ShotCalculator.AimingParameters;
import frc.robot.util.LoggedTunableNumber;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private static final LoggedTunableNumber KP = new LoggedTunableNumber("Hood/Kp");
  private static final LoggedTunableNumber KD = new LoggedTunableNumber("Hood/Kd");
  private static final LoggedTunableNumber MAX_VELOCITY =
      new LoggedTunableNumber("Hood/Max Velocity");
  private static final LoggedTunableNumber MAX_ACCELERATION =
      new LoggedTunableNumber("Hood/Max Acceleration");

  private static final LoggedTunableNumber STOWED_POSITION =
      new LoggedTunableNumber("Hood/Stowed Position");

  private static final LoggedTunableNumber AMP_POSITION =
      new LoggedTunableNumber("Hood/Amp Position");

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private final ProfiledPIDController profiledFeedback;

  static {
    switch (Constants.ROBOT) {
      case SNAPBACK:
        KP.initDefault(0.0);
        KD.initDefault(0.0);
        MAX_VELOCITY.initDefault(0.0);
        MAX_ACCELERATION.initDefault(0.0);
        STOWED_POSITION.initDefault(20.0);
        AMP_POSITION.initDefault(0.0);
        break;
      case ROBOT_2K24_TEST:
        KP.initDefault(0.0);
        KD.initDefault(0.0);
        MAX_VELOCITY.initDefault(0.0);
        MAX_ACCELERATION.initDefault(0.0);
        STOWED_POSITION.initDefault(20.0);
        AMP_POSITION.initDefault(0.0);

        break;
      case ROBOT_SIM:
        KP.initDefault(90.0);
        KD.initDefault(0.01);
        MAX_VELOCITY.initDefault(1000.0);
        MAX_ACCELERATION.initDefault(1000.0);
        STOWED_POSITION.initDefault(Units.degreesToRadians(38.0));
        AMP_POSITION.initDefault(Units.degreesToRadians(15.0));
        break;
      default:
        break;
    }
  }

  public Hood(HoodIO io) {
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
    Logger.processInputs("Hood", inputs);

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

    Logger.recordOutput("Hood/goal", profiledFeedback.getGoal().position);
    Logger.recordOutput("Hood/setpoint", profiledFeedback.getSetpoint().position);
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

  public Command setPosition(
      Supplier<Optional<Translation2d>> robotPoseSupplier,
      Supplier<Translation2d> velocitySupplier) {
    return runEnd(
        () -> {
          if (robotPoseSupplier.get().isPresent()) {
            AimingParameters aimingParameters =
                ShotCalculator.calculate(robotPoseSupplier.get().get(), velocitySupplier.get());
            setPosition(aimingParameters.shooterAngle().getRadians());
          }
        },
        () -> setPosition(STOWED_POSITION.get()));
  }
}
