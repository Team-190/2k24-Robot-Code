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
    public Rotation2d leftPosition;
    public double leftVelocityRadiansPerSecond;
    public double leftAppliedVolts;
    public double leftCurrentAmps;
    public double leftTemperatureCelsius;

    public Rotation2d rightPosition;
    public double rightVelocityRadiansPerSecond;
    public double rightAppliedVolts;
    public double rightCurrentAmps;
    public double rightTemperatureCelsius;
  }

  /** Updates AdvantageKit inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Sets motor voltage for left climber. */
  public default void setLeftVoltage(double volts) {}

  /** Sets motor voltage for right climber. */
  public default void setRightVoltage(double volts) {}

  /** Sets motor closed loop position setpoint for left climber. */
  public default void setLeftPositionSetpoint(Rotation2d position) {}

  /** Sets motor closed loop position setpoint for right climber. */
  public default void setRightPositionSetpoint(Rotation2d position) {}

  /** Sets motor position for left climber. */
  public default void setLeftPosition(Rotation2d position) {}

  /** Sets motor position for right climber. */
  public default void setRightPosition(Rotation2d position) {}
}
