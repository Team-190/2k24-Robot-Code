package frc.robot.subsystems.snapback.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Interface for Snapback's shooter subsystem. */
public interface ShooterIO {

  /**
   * Inputs for Snapback's shooter subsystem. Positions and velocities are of the roller shafts, not
   * the motor shafts.
   */
  @AutoLog
  public static class ShooterIOInputs {
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

  /** Updates AdvantageKit inputs */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Sets motor voltage for left flywheel. */
  public default void setLeftVoltage(double volts) {}

  /** Sets motor voltage for right flywheel. */
  public default void setRightVoltage(double volts) {}

  /** Sets motor closed loop velocity setpoint for left flywheel. */
  public default void setLeftVelocitySetpoint(double velocityRadiansPerSecond) {}

  /** Sets motor closed loop velocity setpoint for right flywheel. */
  public default void setRightVelocitySetpoint(double velocityRadiansPerSecond) {}
}
