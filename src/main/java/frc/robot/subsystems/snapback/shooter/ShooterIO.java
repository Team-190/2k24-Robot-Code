package frc.robot.subsystems.snapback.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Interface for Snapback's shooter subsystem. */
public interface ShooterIO {

  /**
   * Inputs for Snapback's shooter subsystem. Positions and velocity are of the roller shafts, not
   * the motor shafts
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

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setLeftVoltage(double volts) {}

  public default void setRightVoltage(double volts) {}

  public default void setLeftVelocitySetpoint(double velocityRadiansPerSecond) {}

  public default void setRightVelocitySetpoint(double velocityRadiansPerSecond) {}
}
