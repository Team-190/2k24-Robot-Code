package frc.robot.subsystems.whiplash.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.RobotState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class WhiplashShooter extends SubsystemBase {
  private final WhiplashShooterIO io;
  private final WhiplashShooterIOInputsAutoLogged inputs;
  private double topVelocitySetPointRadiansPerSecond;
  private double bottomVelocitySetPointRadiansPerSecond;

  private boolean isClosedLoop;
  private final SysIdRoutine characterizationRoutine;

  public WhiplashShooter(WhiplashShooterIO io) {
    this.io = io;
    inputs = new WhiplashShooterIOInputsAutoLogged();

    topVelocitySetPointRadiansPerSecond = 0.0;
    isClosedLoop = true;

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(3.5),
                Seconds.of(10),
                (state) -> Logger.recordOutput("Shooter/sysIDState", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.setVoltage(volts.in(Volts)), null, this));
  }

  /**
   * Performs periodic updates for the shooter subsystem. This method is called every 20ms by the
   * WPILib framework. It updates the input values from the shooter IO, processes the inputs for
   * logging, sets the velocity setpoint for the top and bottom motors if closed-loop control is
   * enabled, and updates the PID, feedforward, and motion profile configurations based on tunable
   * values.
   */
  @Override
  public void periodic() {
    switch (Constants.ROBOT) {
      case WHIPLASH:
      case WHIPLASH_SIM:
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        if (isClosedLoop) {
          io.setTopVelocitySetPoint(topVelocitySetPointRadiansPerSecond);
          io.setBottomVelocitySetPoint(-bottomVelocitySetPointRadiansPerSecond);
        }

        Logger.recordOutput("Shooter/Position", inputs.topPosition.getRadians());
        Logger.recordOutput(
            "Shooter/Error", inputs.topVelocityGoalRadiansPerSec - inputs.topVelocityRadPerSec);
        break;
      default:
        break;
    }
  }

  /**
   * Sets the shooter's velocity to the dynamic feed speed. This command will run once and set the
   * velocity setpoint to the feed speed obtained from the {@link RobotState#getControlData()}
   * method. It will also enable the closed-loop control of the shooter motors.
   *
   * @return a command to set the shooter's velocity to the feed speed
   */
  public Command setFeedVelocity() {

    return Commands.runOnce(
        () -> {
          topVelocitySetPointRadiansPerSecond = WhiplashShooterConstants.FEED_SPEED.get();
          bottomVelocitySetPointRadiansPerSecond = topVelocitySetPointRadiansPerSecond;
          isClosedLoop = true;
        });
  }

  /**
   * Sets the shooter's velocity to the pre-defined amplifier speed. This command will run once and
   * set the velocity setpoint to the amplifier speed. It will also enable the closed-loop control
   * of the shooter motors.
   *
   * @return a command to set the shooter's velocity to the amplifier speed
   */
  public Command setAmpVelocity() {

    return Commands.runOnce(
        () -> {
          topVelocitySetPointRadiansPerSecond = WhiplashShooterConstants.TOP_AMP_SPEED.get();
          bottomVelocitySetPointRadiansPerSecond = WhiplashShooterConstants.BOTTOM_AMP_SPEED.get();
          isClosedLoop = true;
        });
  }

  /**
   * Sets the shooter's velocity to the pre-defined subwoofer speed. This command will run once and
   * set the velocity setpoint to the subwoofer speed. It will also enable the closed-loop control
   * of the shooter motors.
   *
   * @return a command to set the shooter's velocity to the subwoofer speed
   */
  public Command setSpeakerVelocity() {
    return Commands.runOnce(
        () -> {
          topVelocitySetPointRadiansPerSecond = WhiplashShooterConstants.SPEAKER_SPEED.get();
          bottomVelocitySetPointRadiansPerSecond = topVelocitySetPointRadiansPerSecond;
          isClosedLoop = true;
        });
  }

  /**
   * Checks if the shooter motors are at the velocity setpoint. This method is used to determine if
   * the shooter is at the desired speed.
   *
   * @return true if the shooter motors are at the velocity setpoint, false otherwise. The method
   *     returns the result of the {@link WhiplashShooterIO#atSetPoint()} method.
   */
  @AutoLogOutput(key = "Shooter/at setpoint")
  public boolean atSetPoint() {
    return io.atSetPoint();
  }

  /**
   * Runs quasistatic and dynamic tests with the motors moving both forwards and backwards to
   * calculate the feedForward gains.
   *
   * @return the feedFoward gains calculated by the tests
   */
  public Command runCharacterization() {
    return Commands.sequence(
        Commands.runOnce(() -> isClosedLoop = false),
        characterizationRoutine.quasistatic(Direction.kForward),
        Commands.waitSeconds(5),
        characterizationRoutine.quasistatic(Direction.kReverse),
        Commands.waitSeconds(5),
        characterizationRoutine.dynamic(Direction.kForward),
        Commands.waitSeconds(5),
        characterizationRoutine.dynamic(Direction.kReverse));
  }

  public void setPID(double kp, double kd) {
    io.setPID(kp, 0.0, kd);
  }

  public void setFeedforward(double ks, double kv, double ka) {
    io.setFeedForward(ks, kv, ka);
  }

  public void setProfile(double maxAcceleration) {
    io.setProfile(maxAcceleration);
  }
}
