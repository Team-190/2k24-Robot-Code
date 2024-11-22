package frc.robot.subsystems.snapback.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.snapback.shooter.ShooterConstants.Goal;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs;
  private Goal goal = Goal.IDLE;

  private final SysIdRoutine sysIdRoutineLeft;
  private final SysIdRoutine sysIdRoutineRight;

  public Shooter(ShooterIO io) {
    this.io = io;
    inputs = new ShooterIOInputsAutoLogged();

    sysIdRoutineLeft =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(3.5),
                Seconds.of(10.0),
                (state) -> Logger.recordOutput("Shooter/sysID State Left", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.setLeftVoltage(volts.in(Volts)), null, this));

    sysIdRoutineRight =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(3.5),
                Seconds.of(10.0),
                (state) -> Logger.recordOutput("Shooter/sysID State Left", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.setRightVoltage(volts.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    io.setLeftVelocityGoal(goal.getLeftGoal());
    io.setRightVelocityGoal(goal.getRightGoal());
  }

  public Command setGoal(Goal goal) {
    return runOnce(() -> this.goal = goal);
  }

  public Goal getGoal() {
    return goal;
  }

  public Command shoot(Goal goal) {
    return setGoal(goal).andThen(runOnce(() -> io.setAcceleratorVoltage(12.0)));
  }

  public boolean atGoal() {
    return io.atGoal();
  }

  public Command runSysId() {
    return Commands.sequence(
        sysIdRoutineLeft
            .quasistatic(Direction.kForward)
            .alongWith(sysIdRoutineLeft.quasistatic(Direction.kForward)),
        Commands.waitSeconds(4),
        sysIdRoutineRight
            .quasistatic(Direction.kReverse)
            .alongWith(sysIdRoutineRight.quasistatic(Direction.kReverse)),
        Commands.waitSeconds(4),
        sysIdRoutineLeft
            .dynamic(Direction.kForward)
            .alongWith(sysIdRoutineLeft.dynamic(Direction.kForward)),
        Commands.waitSeconds(4),
        sysIdRoutineRight
            .dynamic(Direction.kReverse)
            .alongWith(sysIdRoutineRight.dynamic(Direction.kReverse)));
  }
}
