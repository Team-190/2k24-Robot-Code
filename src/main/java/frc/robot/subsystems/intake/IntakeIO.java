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

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setRollersVoltage(double volts) {}

  public default void setIntakePosition(Value position) {}

  public default void toggleIntakePosition() {}
}
