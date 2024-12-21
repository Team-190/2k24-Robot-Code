package frc.robot.subsystems.whiplash.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface WhiplashShooterIO {

  /** The ShooterIOInputs class */
  @AutoLog
  public static class WhiplashShooterIOInputs {

    public Rotation2d topPosition = new Rotation2d();
    public double topVelocityRadPerSec = 0.0;
    public double topAppliedVolts = 0.0;
    public double topCurrentAmps = 0.0;
    public double topTemperatureCelsius = 0.0;

    public Rotation2d bottomPosition = new Rotation2d();
    public double bottomVelocityRadPerSec = 0.0;
    public double bottomAppliedVolts = 0.0;
    public double bottomCurrentAmps = 0.0;
    public double bottomTemperatureCelsius = 0.0;

    public double topVelocityGoalRadiansPerSec = 0.0;
    public double bottomVelocityGoalRadiansPerSec = 0.0;

    public double topVelocitySetpointRadiansPerSec = 0.0;
    public double bottomVelocitySetpointRadiansPerSec = 0.0;

    public double topVelocityErrorRadiansPerSec = 0.0;
    public double bottomVelocityErrorRadiansPerSec = 0.0;
  }

  public default void updateInputs(WhiplashShooterIOInputs inputs) {}

  public default void setTopVelocitySetPoint(double setPointVelocityRadiansPerSecond) {}

  public default void setBottomVelocitySetPoint(double setPointVelocityRadiansPerSecond) {}

  public default void setVoltage(double volts) {}

  public default void setPID(double kP, double kI, double kD) {}

  public default void setFeedForward(double kS, double kV, double kA) {}

  public default void setProfile(double maxAccelerationRadiansPerSecondSquared) {}

  public default boolean atSetPoint() {

    return false;
  }

  public default void stop() {}
}
