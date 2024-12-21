package frc.robot.subsystems.whiplash.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.subsystems.whiplash.shooter.WhiplashShooterConstants.WhiplashShooterGoal;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class WhiplashShooter extends SubsystemBase {
  private final WhiplashShooterIO io;
  private final WhiplashShooterIOInputsAutoLogged inputs;

  private WhiplashShooterGoal goal;

  private boolean isClosedLoop;
  private final SysIdRoutine characterizationRoutine;

  public WhiplashShooter(WhiplashShooterIO io) {
    this.io = io;
    inputs = new WhiplashShooterIOInputsAutoLogged();

    goal = WhiplashShooterGoal.IDLE;

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

  @Override
  public void periodic() {
    switch (Constants.ROBOT) {
      case WHIPLASH:
      case WHIPLASH_SIM:
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        if (isClosedLoop) {
          io.setTopVelocity(goal.getTopGoal());
          io.setBottomVelocity(-goal.getBottomGoal());
        }
        break;
      default:
        break;
    }
  }

  public Command setGoal(WhiplashShooterGoal goal) {
    isClosedLoop = true;
    return Commands.runOnce(() -> this.goal = goal);
  }

  @AutoLogOutput(key = "Shooter/at setpoint")
  public boolean atGoal() {
    return io.atGoal();
  }

  public void setPID(double kp, double kd) {
    io.setPID(kp, 0.0, kd);
  }

  public void setFeedforward(double ks, double kv, double ka) {
    io.setFeedforward(ks, kv, ka);
  }

  public void setProfile(
      double maxAccelerationRadiansPerSecondSquared, double goalToleranceRadiansPerSecond) {
    io.setProfile(maxAccelerationRadiansPerSecondSquared, goalToleranceRadiansPerSecond);
  }

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
}
