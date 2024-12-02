package frc.robot.subsystems.whiplash.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.whiplash.shooter.ShooterConstants.Goal;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs;
  private Goal goal = Goal.IDLE;
  private boolean isClosedLoop;

  private final SysIdRoutine sysIdRoutineBottom;
  private final SysIdRoutine sysIdRoutineTop;

  public Shooter(ShooterIO io) {
    this.io = io;
    inputs = new ShooterIOInputsAutoLogged();
    isClosedLoop = true;

    sysIdRoutineBottom = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.2).per(Second),
            Volts.of(3.5),
            Seconds.of(10.0),
            (state) -> Logger.recordOutput("Shooter/sysID State Bottom", state.toString())),
        new SysIdRoutine.Mechanism(
            (volts) -> io.setBottomVoltage(volts.in(Volts)), null, this));

    sysIdRoutineTop = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.2).per(Second),
            Volts.of(3.5),
            Seconds.of(10.0),
            (state) -> Logger.recordOutput("Shooter/sysID State Top", state.toString())),
        new SysIdRoutine.Mechanism((volts) -> io.setTopVoltage(volts.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    if (isClosedLoop) {
      io.setBottomVelocityGoal(goal.getBottomGoal());
      io.setTopVelocityGoal(goal.getTopGoal());
    }
  }

  public Command setGoal(Goal goal) {
    return runOnce(() -> {
      this.goal = goal;
      isClosedLoop = true;
    });
  }

  @AutoLogOutput(key = "Shooter/Goal")
  public Goal getGoal() {
    return goal;
  }

  public Command shoot(Goal goal) {
    return setGoal(goal);
  }

  @AutoLogOutput(key = "Shooter/AtGoal")
  public boolean atGoal() {
    return io.atGoal();
  }

  public Command runSysId() {
    return Commands.sequence(
        runOnce(() -> isClosedLoop = false),
        sysIdRoutineBottom
            .quasistatic(Direction.kForward)
            .alongWith(sysIdRoutineBottom.quasistatic(Direction.kForward)),
        Commands.waitSeconds(4),
        sysIdRoutineTop
            .quasistatic(Direction.kReverse)
            .alongWith(sysIdRoutineTop.quasistatic(Direction.kReverse)),
        Commands.waitSeconds(4),
        sysIdRoutineBottom
            .dynamic(Direction.kForward)
            .alongWith(sysIdRoutineBottom.dynamic(Direction.kForward)),
        Commands.waitSeconds(4),
        sysIdRoutineTop
            .dynamic(Direction.kReverse)
            .alongWith(sysIdRoutineTop.dynamic(Direction.kReverse)));
  }
}
