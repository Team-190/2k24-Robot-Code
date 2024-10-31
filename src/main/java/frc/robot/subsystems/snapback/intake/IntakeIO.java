package frc.robot.subsystems.snapback.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import org.littletonrobotics.junction.AutoLog;

/** Interface for Snapback's intake subsystem. */
public interface IntakeIO {

  /**
   * Inputs for Snapback's intake subsystem. Positions and velocities are of the roller shafts, not
   * the motor shafts.
   */
  @AutoLog
  public static class IntakeIOInputs {
    public Rotation2d intakePosition = new Rotation2d();
    public double intakeVelocityRadiansPerSecond = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeCurrentAmps = 0.0;
    public double intakeTemperatureCelsius = 0.0;

    public Rotation2d serializerPosition = new Rotation2d();
    public double serializerVelocityRadiansPerSecond = 0.0;
    public double serializerAppliedVolts = 0.0;
    public double serializerCurrentAmps = 0.0;
    public double serializerTemperatureCelsius = 0.0;

    public Rotation2d kickerPosition = new Rotation2d();
    public double kickerVelocityRadiansPerSecond = 0.0;
    public double kickerAppliedVolts = 0.0;
    public double kickerCurrentAmps = 0.0;
    public double kickerTemperatureCelsius = 0.0;

    public boolean sensorValue = false;
    public Value pneumaticValue = Value.kOff;
  }

  /** Updates AdvantageKit inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Sets intake motor voltage. */
  public default void setIntakeVoltage(double volts) {}

  /** Sets serializer motor voltage. */
  public default void setSerializerVoltage(double volts) {}

  /** Sets kicker motor voltage. */
  public default void setKickerVoltage(double volts) {}

  public default void setActuatorValue(Value value) {}
}
