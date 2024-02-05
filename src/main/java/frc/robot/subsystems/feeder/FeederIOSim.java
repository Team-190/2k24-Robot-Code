package frc.robot.subsystems.feeder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class FeederIOSim implements FeederIO {
  private DCMotorSim upperMotorSim = new DCMotorSim(DCMotor.getKrakenX60(1), 2.0, 0.004);
  private DCMotorSim lowerMotorSim = new DCMotorSim(DCMotor.getKrakenX60(1), 2.0, 0.004);

  private double upperAppliedVolts = 0.0;
  private double lowerAppliedVolts = 0.0;

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    upperMotorSim.update(Constants.LOOP_PERIOD_SECS);
    lowerMotorSim.update(Constants.LOOP_PERIOD_SECS);

    inputs.upperPositionRad = upperMotorSim.getAngularPositionRad();
    inputs.upperVelocityRadPerSec = upperMotorSim.getAngularVelocityRadPerSec();
    inputs.upperAppliedVolts = upperAppliedVolts;
    inputs.upperCurrentAmps = new double[] {Math.abs(upperMotorSim.getCurrentDrawAmps())};
    inputs.upperTempCelcius = new double[] {};

    inputs.lowerPositionRad = lowerMotorSim.getAngularPositionRad();
    inputs.lowerVelocityRadPerSec = lowerMotorSim.getAngularVelocityRadPerSec();
    inputs.lowerAppliedVolts = lowerAppliedVolts;
    inputs.lowerCurrentAmps = new double[] {Math.abs(lowerMotorSim.getCurrentDrawAmps())};
    inputs.lowerTempCelcius = new double[] {};
  }

  @Override
  public void setVoltage(double volts) {
    upperAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    lowerAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);

    upperMotorSim.setInputVoltage(upperAppliedVolts);
    lowerMotorSim.setInputVoltage(lowerAppliedVolts);
  }
}
