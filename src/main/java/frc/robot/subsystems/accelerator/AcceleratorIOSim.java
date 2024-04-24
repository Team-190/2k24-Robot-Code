package frc.robot.subsystems.accelerator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class AcceleratorIOSim implements AcceleratorIO {
  private DCMotorSim motorSim = new DCMotorSim(DCMotor.getKrakenX60(1), 2.0, 0.004);

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(AcceleratorIOInputs inputs) {
    motorSim.update(Constants.LOOP_PERIOD_SECS);

    inputs.positionRad = motorSim.getAngularPositionRad();
    inputs.velocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {Math.abs(motorSim.getCurrentDrawAmps())};
    inputs.tempCelcius = new double[] {};
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    motorSim.setInputVoltage(appliedVolts);
  }
}
