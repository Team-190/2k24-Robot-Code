package frc.robot.subsystems.whiplash.climber;

import org.littletonrobotics.junction.AutoLog;

/** Interface for Snapback's climber subsystem. */
public interface ClimberIO {

  /**
   * Inputs for Snapback's climber subsystem. Positions and velocities are of the climber pulleys,
   * not the motor shafts.
   */
  @AutoLog
  public static class ClimberIOInputs {
    public double climberPositionMeters = 0.0;
    public double climberVelocityMetersPerSecond = 0.0;
    public double climberAppliedVolts = 0.0;
    public double climberCurrentAmps = 0.0;
    public double climberTemperatureCelsius = 0.0;
  }

  /** Updates AdvantageKit inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Sets motor voltage for climber. */
  public default void setClimberVoltage(double volts) {}
}

