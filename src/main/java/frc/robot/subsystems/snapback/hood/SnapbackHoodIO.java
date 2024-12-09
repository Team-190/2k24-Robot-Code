package frc.robot.subsystems.snapback.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Interface for Snapback's hood subsystem. */
public interface SnapbackHoodIO {

  /**
   * Inputs for Snapback's hood subsystem. Positions and velocities are of the output shaft, not the
   * motor shaft.
   */
  @AutoLog
  public static class SnapbackHoodIOInputs {
    public Rotation2d position = new Rotation2d();
    public double velocityRadiansPerSecond = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public Rotation2d positionGoal;
    public Rotation2d positionSetpoint;
    public Rotation2d positionError;
  }

  /** Updates AdvantageKit inputs. */
  public default void updateInputs(SnapbackHoodIOInputs inputs) {}

  /** Sets motor voltage. */
  public default void setVoltage(double volts) {}

  /** Sets motor closed loop position setpoint. */
  public default void setPositionGoal(Rotation2d position) {}

  /** Sets motor position. */
  public default void setPosition(Rotation2d position) {}

  /** Checks if the hood is within tolerance */
  public default boolean atGoal() {
    return false;
  }
}
