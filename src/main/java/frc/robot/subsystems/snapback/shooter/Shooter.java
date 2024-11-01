package frc.robot.subsystems.snapback.shooter;

import frc.robot.RobotState;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;

public class Shooter {

  @RequiredArgsConstructor
  public enum Goal {
    IDLE(() -> 0.0, () -> 0.0),
    SPEAKER(
        () -> RobotState.getControlData().speakerShooterSpeed().f1Speed(),
        () -> RobotState.getControlData().speakerShooterSpeed().f2Speed()),
    FEED(
        () -> RobotState.getControlData().ampFeedShooterSpeed().f2Speed(),
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
}
