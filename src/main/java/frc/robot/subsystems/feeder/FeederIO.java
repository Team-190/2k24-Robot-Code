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

    public double lowerPositionRad = 0.0;
    public double lowerVelocityRadPerSec = 0.0;
    public double lowerAppliedVolts = 0.0;
    public double[] lowerCurrentAmps = new double[] {};
    public double[] lowerTempCelcius = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FeederIOInputs inputs) {}

  /** Run the upper feeder motor at the specified voltage. */
  public default void setUpperVoltage(double volts) {}

  /** Run the lower feeder motor at the specified voltage. */
  public default void setLowerVoltage(double volts) {}
}
