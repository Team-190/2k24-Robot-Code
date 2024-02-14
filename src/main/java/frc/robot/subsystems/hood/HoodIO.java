package frc.robot.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public Rotation2d position = new Rotation2d();
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setVoltage(double volts) {}
}
