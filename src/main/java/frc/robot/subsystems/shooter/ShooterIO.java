package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double leftPositionRad = 0.0;
    public double leftVelocityRadPerSec = 0.0;
    public double leftAppliedVolts = 0.0;
    public double[] leftCurrentAmps = new double[] {};
    public double[] leftTempCelcius = new double[] {};

    public double rightPositionRad = 0.0;
    public double rightVelocityRadPerSec = 0.0;
    public double rightAppliedVolts = 0.0;
    public double[] rightCurrentAmps = new double[] {};
    public double[] rightTempCelcius = new double[] {};
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setLeftVoltage(double volts) {}

  public default void setRightVoltage(double volts) {}
}
