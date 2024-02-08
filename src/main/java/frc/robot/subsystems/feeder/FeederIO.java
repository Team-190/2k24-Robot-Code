package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public double upperPositionRad = 0.0;
    public double upperVelocityRadPerSec = 0.0;
    public double upperAppliedVolts = 0.0;
    public double[] upperCurrentAmps = new double[] {};
    public double[] upperTempCelcius = new double[] {};

    public double lowerLeftPositionRad = 0.0;
    public double lowerLeftVelocityRadPerSec = 0.0;
    public double lowerLeftAppliedVolts = 0.0;
    public double[] lowerLeftCurrentAmps = new double[] {};
    public double[] lowerLeftTempCelcius = new double[] {};

    public double lowerRightPositionRad = 0.0;
    public double lowerRightVelocityRadPerSec = 0.0;
    public double lowerRightAppliedVolts = 0.0;
    public double[] lowerRightCurrentAmps = new double[] {};
    public double[] lowerRightTempCelcius = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FeederIOInputs inputs) {}

  /** Run the upper feeder motor at the specified voltage. */
  public default void setUpperVoltage(double volts) {}

  /** Run the lower feeder motor at the specified voltage. */
  public default void setLowerVoltage(double volts) {}
}
