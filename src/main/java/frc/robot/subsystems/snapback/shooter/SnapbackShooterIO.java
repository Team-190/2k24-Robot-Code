package frc.robot.subsystems.snapback.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Interface for Snapback's shooter subsystem. */
public interface SnapbackShooterIO {

  /**
   * Inputs for Snapback's shooter subsystem. Positions and velocities are of the roller shafts, not
   * the motor shafts.
   */
  @AutoLog
  public static class SnapbackShooterIOInputs {
    public Rotation2d leftPosition = new Rotation2d();
    public double leftVelocityRadiansPerSecond = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftCurrentAmps = 0.0;
    public double leftTemperatureCelsius = 0.0;
    public double leftVelocityGoalRadiansPerSecond = 0.0;
    public double leftVelocitySetpointRadiansPerSecond = 0.0;
    public double leftVelocityErrorRadiansPerSecond = 0.0;

    public Rotation2d rightPosition = new Rotation2d();
    public double rightVelocityRadiansPerSecond = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightCurrentAmps = 0.0;
    public double rightTemperatureCelsius = 0.0;
    public double rightVelocityGoalRadiansPerSecond = 0.0;
    public double rightVelocitySetpointRadiansPerSecond = 0.0;
    public double rightVelocityErrorRadiansPerSecond = 0.0;

    public Rotation2d acceleratorPosition = new Rotation2d();
    public double acceleratorVelocityRadiansPerSecond = 0.0;
    public double acceleratorAppliedVolts = 0.0;
    public double acceleratorCurrentAmps = 0.0;
    public double acceleratorTemperatureCelsius = 0.0;
  }

  /** Updates AdvantageKit inputs */
  public default void updateInputs(SnapbackShooterIOInputs inputs) {}

  /** Sets motor voltage for left flywheel. */
  public default void setLeftVoltage(double volts) {}

  /** Sets motor voltage for right flywheel. */
  public default void setRightVoltage(double volts) {}

  /** Sets motor voltage for accelerator motor */
  public default void setAcceleratorVoltage(double volts) {}

  /** Sets motor closed loop velocity setpoint for left flywheel. */
  public default void setLeftVelocity(double velocityRadiansPerSecond) {}

  /** Sets motor closed loop velocity setpoint for right flywheel. */
  public default void setRightVelocity(double velocityRadiansPerSecond) {}

  public default void setPID(double kp, double ki, double kd) {}

  public default void setFeedforward(double ks, double kv, double ka) {}

  public default void setProfile(
      double maxAccelerationRadiansPerSecondSquared, double goalToleranceRadians) {}

  /** Returns true if the shooter is at the goal velocity. */
  public default boolean atGoal() {
    return false;
  }
}
