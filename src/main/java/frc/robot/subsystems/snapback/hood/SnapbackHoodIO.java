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
    public Rotation2d positionGoal = new Rotation2d();
    public Rotation2d positionSetpoint = new Rotation2d();
    public Rotation2d positionError = new Rotation2d();
  }

  /** Updates AdvantageKit inputs. */
  public default void updateInputs(SnapbackHoodIOInputs inputs) {}

  /** Sets motor voltage. */
  public default void setVoltage(double volts) {}

  /** Sets motor closed loop position setpoint. */
  public default void setPosition(Rotation2d position) {}

  public default void setPID(double kp, double ki, double kd) {}

  public default void setFeedforward(double ks, double kv, double ka) {}

  public default void setProfile(
      double maxVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared,
      double goalToleranceRadians) {}
  /** Checks if the hood is within tolerance */
  public default boolean atGoal() {
    return false;
  }
}
