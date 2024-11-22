package frc.robot.subsystems.snapback.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Interface for Snapback's climber subsystem. */
public interface ClimberIO {

  /**
   * Inputs for Snapback's climber subsystem. Positions and velocities are of the climber pulleys,
   * not the motor shafts.
   */
  @AutoLog
  public static class ClimberIOInputs {
    public double leftPosition = 0.0;
    public double leftVelocityRadiansPerSecond = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftCurrentAmps = 0.0;
    public double leftTemperatureCelsius = 0.0;
    public double leftPositionGoalMeters = 0.0;
    public double leftPositionErrorMeters = 0.0;

    public double rightPosition = 0.0;
    public double rightVelocityRadiansPerSecond = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightCurrentAmps = 0.0;
    public double rightTemperatureCelsius = 0.0;
    public double rightPositionGoalMeters = 0.0;
    public double rightPositionErrorMeters = 0.0;
  }

  /** Updates AdvantageKit inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Sets motor voltage for left climber. */
  public default void setLeftVoltage(double volts) {}

  /** Sets motor voltage for right climber. */
  public default void setRightVoltage(double volts) {}

  /** Sets motor closed loop position setpoint for left climber. */
  public default void setLeftPositionGoal(double positionMeters) {}

  /** Sets motor closed loop position setpoint for right climber. */
  public default void setRightPositionGoal(double positionMeters) {}

  /** Sets motor position for left climber. */
  public default void setLeftPosition(double positionMeters) {}

  /** Sets motor position for right climber. */
  public default void setRightPosition(double positionMeters) {}

  /** Returns true if the climber is at the goal position. */
  public default boolean atGoal() {
    return false;
  }
}
