package frc.robot.subsystems.snapback.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Interface for Snapback's hood subsystem. */
public interface HoodIO {

  /**
   * Inputs for Snapback's hood subsystem. Positions and velocities are of the output shaft, not the
   * motor shaft.
   */
  @AutoLog
  public static class HoodIOInputs {
    public Rotation2d position;
    public double velocityRadiansPerSecond;
    public double appliedVolts;
    public double currentAmps;
    public double temperatureCelsius;
  }

  /** Updates AdvantageKit inputs. */
  public default void updateInputs(HoodIOInputs inputs) {}

  /** Sets motor voltage. */
  public default void setVoltage(double volts) {}

  /** Sets motor closed loop position setpoint. */
  public default void setPositionSetpoint(Rotation2d position) {}

  /** Sets motor position. */
  public default void setPosition(Rotation2d position) {}
}
