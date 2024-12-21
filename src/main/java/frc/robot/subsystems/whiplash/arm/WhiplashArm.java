package frc.robot.subsystems.whiplash.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.subsystems.whiplash.arm.WhiplashArmConstants.WhiplashArmGoal;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class WhiplashArm extends SubsystemBase {
  private final WhiplashArmIOInputsAutoLogged inputs;
  private final WhiplashArmIO io;
  private Rotation2d positionGoal;
  private boolean isClosedLoop;
  private boolean isAmping;
  private boolean isSlowMode;
  private final SysIdRoutine characterizationRoutine;

  public WhiplashArm(WhiplashArmIO io) {
    inputs = new WhiplashArmIOInputsAutoLogged();
    this.io = io;
    positionGoal = WhiplashArmGoal.STOW.getAngle();
    isClosedLoop = true;

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Second),
                Volts.of(3.5),
                Seconds.of(10),
                (state) -> Logger.recordOutput("Arm/sysIDState", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.setVoltage(volts.in(Volts)), null, this));
  }

  /**
   * This method is called periodically during the robot's main loop. It updates the arm's input
   * values, processes the inputs for logging, and sets the arm position based on the desired angle
   * if closed-loop control is enabled. Additionally, it updates the PID, feedforward, and profile
   * settings for the arm control.
   */
  @Override
  public void periodic() {
    switch (Constants.ROBOT) {
      case WHIPLASH:
      case WHIPLASH_SIM:
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        if (isClosedLoop) {
          if (isAmping) {
            if (!isSlowMode) {
              isSlowMode = true;
              io.setProfile(10, 10, WhiplashArmConstants.CONSTRAINTS.goalToleranceRadians().get());
            }
            io.setPosition(positionGoal);
          }
          if (!isAmping) {
            if (isSlowMode) {
              io.setProfile(
                  WhiplashArmConstants.CONSTRAINTS.maxVelocityRadiansPerSecond().get(),
                  WhiplashArmConstants.CONSTRAINTS.maxAccelerationRadiansPerSecondSqaured().get(),
                  WhiplashArmConstants.CONSTRAINTS.goalToleranceRadians().get());
              isSlowMode = false;
            }
            io.setPosition(positionGoal);
          }
        }
        break;
      default:
        break;
    }
  }

  public Command setGoal(WhiplashArmGoal goal) {
    isAmping = goal.equals(WhiplashArmGoal.AMP) ? true : false;
    isClosedLoop = true;
    return Commands.runOnce(() -> positionGoal = goal.getAngle());
  }

  @AutoLogOutput(key = "Arm/At Goal")
  public boolean atGoal() {
    return io.atGoal();
  }

  public Command runQuasistaticCharacterization(Direction direction) {
    return Commands.sequence(
        Commands.runOnce(() -> isClosedLoop = false),
        characterizationRoutine.quasistatic(direction));
  }

  public Command runDynamicCharacterization(Direction direction) {
    return Commands.sequence(
        Commands.runOnce(() -> isClosedLoop = false), characterizationRoutine.dynamic(direction));
  }

  public Command runVoltage(double volts) {
    return Commands.run(() -> io.setVoltage(volts));
  }

  public Rotation2d getPosition() {
    return inputs.absolutePosition;
  }

  public void setPID(double kp, double kd) {
    io.setPID(kp, 0.0, kd);
  }

  public void setFeedforward(double ks, double kg, double kv) {
    io.setFeedforward(ks, kg, kv);
  }

  public void setProfile(
      double maxVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared,
      double goalToleranceRadians) {
    io.setProfile(
        maxVelocityRadiansPerSecond, maxAccelerationRadiansPerSecondSquared, goalToleranceRadians);
  }
}
