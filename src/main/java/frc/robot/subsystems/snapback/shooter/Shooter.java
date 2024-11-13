package frc.robot.subsystems.snapback.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs;
  private Goal goal = Goal.IDLE;

  public Shooter(ShooterIO io) {
    this.io = io;
    inputs = new ShooterIOInputsAutoLogged();
  }

  @RequiredArgsConstructor
  public enum Goal {
    IDLE(() -> 0.0, () -> 0.0),
    SPEAKER(
        () -> RobotState.getControlData().speakerShooterSpeed().f1Speed(),
        () -> RobotState.getControlData().speakerShooterSpeed().f2Speed()),
    FEED(
        () -> RobotState.getControlData().ampFeedShooterSpeed().f1Speed(),
        () -> RobotState.getControlData().ampFeedShooterSpeed().f2Speed());

    private final DoubleSupplier leftGoal;
    private final DoubleSupplier rightGoal;

    private double getLeftGoal() {
      return leftGoal.getAsDouble();
    }

    private double getRightGoal() {
      return rightGoal.getAsDouble();
    }
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

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    io.setLeftVelocityGoal(goal.getLeftGoal());
    io.setRightVelocityGoal(goal.getRightGoal());
  }
}
