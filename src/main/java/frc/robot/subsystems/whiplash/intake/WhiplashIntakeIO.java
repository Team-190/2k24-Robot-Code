package frc.robot.subsystems.whiplash.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface WhiplashIntakeIO {
  @AutoLog
  public static class WhiplashIntakeIOInputs {
    public Rotation2d topPosition = new Rotation2d();
    public double topVelocityRadiansPerSecond = 0.0;
    public double topAppliedVolts = 0.0;
    public double topCurrentAmps = 0.0;
    public double topTemperatureCelsius = 0.0;

    public Rotation2d bottomPosition = new Rotation2d();
    public double bottomVelocityRadiansPerSecond = 0.0;
    public double bottomAppliedVolts = 0.0;
    public double bottomCurrentAmps = 0.0;
    public double bottomTemperatureCelsius = 0.0;

    public Rotation2d acceleratorPosition = new Rotation2d();
    public double acceleratorVelocityRadiansPerSecond = 0.0;
    public double acceleratorAppliedVolts = 0.0;
    public double acceleratorCurrentAmps = 0.0;
    public double acceleratorTemperatureCelsius = 0.0;

    public boolean intakeSensor = false;
    public boolean middleSensor = false;
    public boolean finalSensor = false;
  }

  public default void updateInputs(WhiplashIntakeIOInputs inputs) {}

  public default void setTopVoltage(double volts) {}

  public default void setBottomVoltage(double volts) {}

  public default void setAcceleratorVoltage(double volts) {}

  public default void stop() {}
}
