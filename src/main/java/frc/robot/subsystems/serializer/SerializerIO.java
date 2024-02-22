package frc.robot.subsystems.serializer;

import org.littletonrobotics.junction.AutoLog;

public interface SerializerIO {
  @AutoLog
  public static class SerializerIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};
  }

  public default void updateInputs(SerializerIOInputs inputs) {}

  public default void setVoltage(double volts) {}
}
