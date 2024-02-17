package frc.robot.subsystems.amp;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface AmpIO {
  @AutoLog
  public static class AmpIOInputs {
    public Rotation2d position = new Rotation2d();
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};
  }

  public default void updateInputs(AmpIOInputs inputs) {
  }
  
  public default void setVoltage(double volts) {}
}
