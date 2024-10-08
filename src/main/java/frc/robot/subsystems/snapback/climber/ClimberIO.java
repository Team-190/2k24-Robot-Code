package frc.robot.subsystems.snapback.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Interface for Snapback's climber subsystem. */
public interface ClimberIO {

  /** Inputs for Snapback's climber subsystem. Positions and velocity are of the climber pulleys, not the motor shafts */
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

  public default void setLeftVoltage(double volts) {}

  public default void setRightVoltage(double volts) {}

  public default void setLeftPosition(Rotation2d position) {}

  public default void setRightPosition(Rotation2d position) {}
}
