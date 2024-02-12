package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double rollersPositionRad = 0.0;
    public double rollersVelocityRadPerSec = 0.0;
    public double rollersAppliedVolts = 0.0;
    public double[] rollersCurrentAmps = new double[] {};
    public double[] rollersTempCelcius = new double[] {};

    public Rotation2d intakePositionRad = new Rotation2d();
    public double intakeVelocityRadPerSec = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double[] intakeCurrentAmps = new double[] {};
    public double[] intakeTempCelcius = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run the rollers motor at the specified voltage. */
  public default void setRollersVoltage(double volts) {}

  /** Run the intake motor at the specified voltage. */
  public default void setIntakeVoltage(double volts) {}
}
