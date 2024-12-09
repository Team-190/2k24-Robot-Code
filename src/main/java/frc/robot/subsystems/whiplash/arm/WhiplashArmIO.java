package frc.robot.subsystems.whiplash.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface WhiplashArmIO {
  @AutoLog
  public static class WhiplashArmIOInputs {

    public Rotation2d armPosition = new Rotation2d();
    public double armVelocityRadPerSec = 0.0;
    public double armAppliedVolts = 0.0;
    public double armCurrentAmps = 0.0;
    public double armTemperatureCelsius = 0.0;
    public Rotation2d armAbsolutePosition = new Rotation2d();

    public Rotation2d positionSetpoint = new Rotation2d();
    public Rotation2d positionError = new Rotation2d();
    public Rotation2d positionGoal = new Rotation2d();
  }

  public default void updateInputs(WhiplashArmIOInputs inputs) {}

  public default void setArmVoltage(double volts) {}

  public default void stop() {}

  public default void setPID(double kp, double ki, double kd) {}

  public default void setFeedforward(double ks, double kg, double kv) {}

  public default void setProfile(double max_velocity, double max_acceleration) {}

  public default void setArmPosition(Rotation2d currentPosition, Rotation2d setpointPosition) {}

  public default boolean atSetpoint() {
    return false;
  }
}
