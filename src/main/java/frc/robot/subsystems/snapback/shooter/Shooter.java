package frc.robot.subsystems.snapback.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.snapback.shooter.ShooterConstants.Goal;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs;
  private Goal goal = Goal.IDLE;

  public Shooter(ShooterIO io) {
    this.io = io;
    inputs = new ShooterIOInputsAutoLogged();
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
}
