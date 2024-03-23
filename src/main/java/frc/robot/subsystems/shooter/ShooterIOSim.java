package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ShooterIOSim implements ShooterIO {
  private DCMotorSim leftMotorSim = new DCMotorSim(DCMotor.getKrakenX60(1), 2.0, 0.004);
  private DCMotorSim rightMotorSim = new DCMotorSim(DCMotor.getKrakenX60(1), 2.0, 0.004);

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    leftMotorSim.update(Constants.LOOP_PERIOD_SECS);
    rightMotorSim.update(Constants.LOOP_PERIOD_SECS);

    inputs.leftPositionRad = leftMotorSim.getAngularPositionRad();
    inputs.leftVelocityRadPerSec = leftMotorSim.getAngularVelocityRadPerSec();
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftCurrentAmps = new double[] {Math.abs(leftMotorSim.getCurrentDrawAmps())};
    inputs.leftTempCelcius = new double[] {};

    inputs.rightPositionRad = rightMotorSim.getAngularPositionRad();
    inputs.rightVelocityRadPerSec = rightMotorSim.getAngularVelocityRadPerSec();
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightCurrentAmps = new double[] {Math.abs(rightMotorSim.getCurrentDrawAmps())};
    inputs.rightTempCelcius = new double[] {};
  }

  @Override
  public void setLeftVoltage(double volts) {
    leftAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);

    leftMotorSim.setInputVoltage(leftAppliedVolts);
  }

  @Override
  public void setRightVoltage(double volts) {
    rightAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);

    rightMotorSim.setInputVoltage(rightAppliedVolts);
  }
}
