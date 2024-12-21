package frc.robot.subsystems.whiplash.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface WhiplashShooterIO {

  /** The ShooterIOInputs class */
  @AutoLog
  public static class WhiplashShooterIOInputs {

    public Rotation2d topPosition = new Rotation2d();
    public double topVelocityRadiansPerSecond = 0.0;
    public double topAppliedVolts = 0.0;
    public double topCurrentAmps = 0.0;
    public double topTemperatureCelsius = 0.0;
    public double topVelocityGoalRadiansPerSecond = 0.0;
    public double topVelocitySetpointRadiansPerSecond = 0.0;
    public double topVelocityErrorRadiansPerSecond = 0.0;

    public Rotation2d bottomPosition = new Rotation2d();
    public double bottomVelocityRadiansPerSecond = 0.0;
    public double bottomAppliedVolts = 0.0;
    public double bottomCurrentAmps = 0.0;
    public double bottomTemperatureCelsius = 0.0;
    public double bottomVelocityGoalRadiansPerSecond = 0.0;
    public double bottomVelocitySetpointRadiansPerSecond = 0.0;
    public double bottomVelocityErrorRadiansPerSecond = 0.0;
  }

  public default void updateInputs(WhiplashShooterIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setTopVelocitySetpoint(double setpointVelocityRadiansPerSecond) {}

  public default void setBottomVelocitySetpoint(double setpointVelocityRadiansPerSecond) {}

  public default void setPID(double kp, double ki, double kd) {}

  public default void setFeedforward(double ks, double kv, double ka) {}

  public default void setProfile(
      double maxAccelerationRadiansPerSecondSquared, double goalToleranceRadiansPerSecond) {}

  public default boolean atSetpoint() {
    return false;
  }

  public default void stop() {}
}
