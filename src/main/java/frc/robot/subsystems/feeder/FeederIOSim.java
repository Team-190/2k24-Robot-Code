package frc.robot.subsystems.feeder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class FeederIOSim implements FeederIO {
  private DCMotorSim upperMotorSim = new DCMotorSim(DCMotor.getKrakenX60(1), 2.0, 0.004);
  private DCMotorSim lowerLeftMotorSim = new DCMotorSim(DCMotor.getKrakenX60(1), 2.0, 0.004);
  private DCMotorSim lowerRightMotorSim = new DCMotorSim(DCMotor.getKrakenX60(1), 2.0, 0.004);

  private double upperAppliedVolts = 0.0;
  private double lowerLeftAppliedVolts = 0.0;
  private double lowerRightAppliedVolts = 0.0;

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    upperMotorSim.update(Constants.LOOP_PERIOD_SECS);
    lowerLeftMotorSim.update(Constants.LOOP_PERIOD_SECS);

    inputs.upperPositionRad = upperMotorSim.getAngularPositionRad();
    inputs.upperVelocityRadPerSec = upperMotorSim.getAngularVelocityRadPerSec();
    inputs.upperAppliedVolts = upperAppliedVolts;
    inputs.upperCurrentAmps = new double[] {Math.abs(upperMotorSim.getCurrentDrawAmps())};
    inputs.upperTempCelcius = new double[] {};

    inputs.lowerLeftPositionRad = lowerLeftMotorSim.getAngularPositionRad();
    inputs.lowerLeftVelocityRadPerSec = lowerLeftMotorSim.getAngularVelocityRadPerSec();
    inputs.lowerLeftAppliedVolts = lowerLeftAppliedVolts;
    inputs.lowerLeftCurrentAmps = new double[] {Math.abs(lowerLeftMotorSim.getCurrentDrawAmps())};
    inputs.lowerLeftTempCelcius = new double[] {};

    inputs.lowerRightPositionRad = lowerRightMotorSim.getAngularPositionRad();
    inputs.lowerRightVelocityRadPerSec = lowerRightMotorSim.getAngularVelocityRadPerSec();
    inputs.lowerRightAppliedVolts = lowerRightAppliedVolts;
    inputs.lowerRightCurrentAmps = new double[] {Math.abs(lowerRightMotorSim.getCurrentDrawAmps())};
    inputs.lowerRightTempCelcius = new double[] {};
  }

  @Override
  public void setUpperVoltage(double volts) {
    upperAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    upperMotorSim.setInputVoltage(upperAppliedVolts);
  }

  @Override
  public void setLowerVoltage(double volts) {
    lowerLeftAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    lowerLeftMotorSim.setInputVoltage(lowerLeftAppliedVolts);

    lowerRightAppliedVolts = -MathUtil.clamp(volts, -12.0, 12.0);
    lowerRightMotorSim.setInputVoltage(lowerRightAppliedVolts);
  }
}
