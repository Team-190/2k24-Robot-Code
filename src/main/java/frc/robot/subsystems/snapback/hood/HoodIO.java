package frc.robot.subsystems.snapback.hood;

import edu.wpi.first.math.geometry.Rotation2d;

/** Interface for Snapback's hood subsystem. */
public interface HoodIO {

  /**
   * Inputs for Snapback's hood subsystem. Positions and Velocities are of the output shaft, not the
   * motor shaft
   */
  public static class HoodIOInputs {
    public Rotation2d position;
    public double velocityRadiansPerSecond;
    public double appliedVolts;
    public double currentAmps;
    public double temperatureCelsius;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setPositionSetpoint(Rotation2d position) {}

  public default void setPosition(Rotation2d position) {}
}
