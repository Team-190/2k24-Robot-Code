package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public Rotation2d position;
    public double velocityRadPerSec;
    public double appliedVolts;
    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};
  }

  public default void updateInputs(PivotIOInputs inputs) {}

  public default void setVoltage(double volts) {}
}
