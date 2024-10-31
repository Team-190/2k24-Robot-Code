package frc.robot.subsystems.snapback.shooter;

import java.util.function.DoubleSupplier;

import lombok.RequiredArgsConstructor;

public class Shooter {

  @RequiredArgsConstructor
  public enum Goal {
    IDLE(() -> 0.0, () -> 0.0),
    SPEAKER(() -> 0.0, () -> 0.0),
    FEED(() -> 0.0, () -> 0.0);

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
