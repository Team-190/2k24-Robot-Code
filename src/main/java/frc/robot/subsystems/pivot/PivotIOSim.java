package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class PivotIOSim implements PivotIO {
  private static final double GEAR_RATIO = 88.0 / 14.0;
  ;
  private SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1), GEAR_RATIO, 0.004, 0.25, 0, Math.PI, true, 0);
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    pivotSim.update(Constants.LOOP_PERIOD_SECS);

    inputs.position = Rotation2d.fromRadians(pivotSim.getAngleRads());
    inputs.velocityRadPerSec = pivotSim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {Math.abs(pivotSim.getCurrentDrawAmps())};
    inputs.tempCelcius = new double[] {};
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    pivotSim.setInputVoltage(appliedVolts);
  }
}
