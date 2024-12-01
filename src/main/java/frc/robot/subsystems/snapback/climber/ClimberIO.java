package frc.robot.subsystems.snapback.climber;

import org.littletonrobotics.junction.AutoLog;

/** Interface for Snapback's climber subsystem. */
public interface ClimberIO {

  /**
   * Inputs for Snapback's climber subsystem. Positions and velocities are of the climber pulleys,
   * not the motor shafts.
   */
  @AutoLog
  public static class ClimberIOInputs {
    public double leftPositionMeters = 0.0;
    public double leftVelocityMetersPerSecond = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftCurrentAmps = 0.0;
    public double leftTemperatureCelsius = 0.0;

    public double rightPositionMeters = 0.0;
    public double rightVelocityMetersPerSecond = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightCurrentAmps = 0.0;
    public double rightTemperatureCelsius = 0.0;
  }

  /** Updates AdvantageKit inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Sets motor voltage for left climber. */
  public default void setLeftVoltage(double volts) {}

  /** Sets motor voltage for right climber. */
  public default void setRightVoltage(double volts) {}
}
