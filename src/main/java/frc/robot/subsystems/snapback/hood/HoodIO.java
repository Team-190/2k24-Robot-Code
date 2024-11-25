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
    public Rotation2d position = Rotation2d.fromRadians(0);
    public double velocityRadiansPerSecond = 0;
    public double appliedVolts = 0;
    public double currentAmps = 0;
    public double temperatureCelsius = 0;
    public Rotation2d positionSetpoint;
    public Rotation2d positionError;
    public Rotation2d positionGoal;
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
