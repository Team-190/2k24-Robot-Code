package frc.robot.subsystems.whiplash.shooter;

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
    public Rotation2d bottomPosition = new Rotation2d();
    public double bottomVelocityRadiansPerSecond = 0.0;
    public double bottomAppliedVolts = 0.0;
    public double bottomCurrentAmps = 0.0;
    public double bottomTemperatureCelsius = 0.0;
    public double bottomVelocityGoalRadiansPerSecond = 0.0;
    public double bottomVelocitySetpointRadiansPerSecond = 0.0;
    public double bottomVelocityErrorRadiansPerSecond = 0.0;

    public Rotation2d topPosition = new Rotation2d();
    public double topVelocityRadiansPerSecond = 0.0;
    public double topAppliedVolts = 0.0;
    public double topCurrentAmps = 0.0;
    public double topTemperatureCelsius = 0.0;
    public double topVelocityGoalRadiansPerSecond = 0.0;
    public double topVelocitySetpointRadiansPerSecond = 0.0;
    public double topVelocityErrorRadiansPerSecond = 0.0;
  }

  /** Updates AdvantageKit inputs */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Sets motor voltage for bottom flywheel. */
  public default void setBottomVoltage(double volts) {}

  /** Sets motor voltage for top flywheel. */
  public default void setTopVoltage(double volts) {}

  /** Sets motor closed loop velocity setpoint for bottom flywheel. */
  public default void setBottomVelocityGoal(double velocityRadiansPerSecond) {}

  /** Sets motor closed loop velocity setpoint for top flywheel. */
  public default void setTopVelocityGoal(double velocityRadiansPerSecond) {}

  /** Returns true if the shooter is at the goal velocity. */
  public default boolean atGoal() {
    return false;
  }
}
