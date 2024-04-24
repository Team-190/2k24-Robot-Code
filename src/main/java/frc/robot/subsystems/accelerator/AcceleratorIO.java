package frc.robot.subsystems.accelerator;

import org.littletonrobotics.junction.AutoLog;

public interface AcceleratorIO {
  @AutoLog
  public static class AcceleratorIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};
  }

  public default void updateInputs(AcceleratorIOInputs inputs) {}

  public default void setVoltage(double volts) {}
}
