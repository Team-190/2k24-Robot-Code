package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double rollersPositionRad = 0.0;
    public double rollersVelocityRadPerSec = 0.0;
    public double rollersAppliedVolts = 0.0;
    public double[] rollersCurrentAmps = new double[] {};
    public double[] rollersTempCelcius = new double[] {};

    public Value leftPosition = Value.kReverse;
    public Value rightPosition = Value.kReverse;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run the rollers motor at the specified voltage. */
  public default void setRollersVoltage(double volts) {}

  /** set the intake to the specified position. */
  public default void setIntakePosition(Value position) {}

  /** toggle the intake to the position it currently is not in */
  public default void toggleIntakePosition() {}
}
