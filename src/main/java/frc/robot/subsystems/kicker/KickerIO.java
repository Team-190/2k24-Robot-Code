package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  public static class KickerIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};
  }

  public default void updateInputs(KickerIOInputs inputs) {}

  public default void setVoltage(double volts) {}
}
