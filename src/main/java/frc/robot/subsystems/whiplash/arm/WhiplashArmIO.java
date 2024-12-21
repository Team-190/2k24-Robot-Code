package frc.robot.subsystems.whiplash.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface WhiplashArmIO {
  @AutoLog
  public static class WhiplashArmIOInputs {
    public Rotation2d absolutePosition = new Rotation2d();
    public Rotation2d position = new Rotation2d();
    public double velocityRadiansPerSecond = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public Rotation2d positionGoal = new Rotation2d();
    public Rotation2d positionSetpoint = new Rotation2d();
    public Rotation2d positionError = new Rotation2d();
  }

  public default void updateInputs(WhiplashArmIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setPosition(Rotation2d positionGoal) {}

  public default void setPID(double kp, double ki, double kd) {}

  public default void setFeedforward(double ks, double kg, double kv) {}

  public default void setProfile(
      double maxVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared,
      double goalToleranceRadians) {}

  public default boolean atGoal() {
    return false;
  }

  public default void stop() {}
}
