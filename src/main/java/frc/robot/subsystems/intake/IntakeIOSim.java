package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {
  private DCMotorSim rollersSim = new DCMotorSim(DCMotor.getKrakenX60(1), 2.0, 0.004);
  private SingleJointedArmSim intakeSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1), 42.0, 0.0042, 0.3048, Math.PI / 2, Math.PI, true, Math.PI / 2);

  private double rollersAppliedVolts = 0.0;
  private double intakeAppliedVolts = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    rollersSim.update(Constants.LOOP_PERIOD_SECS);

    inputs.rollersPositionRad = rollersSim.getAngularPositionRad();
    inputs.rollersVelocityRadPerSec = rollersSim.getAngularVelocityRadPerSec();
    inputs.rollersAppliedVolts = rollersAppliedVolts;
    inputs.rollersCurrentAmps = new double[] {Math.abs(rollersSim.getCurrentDrawAmps())};
    inputs.rollersTempCelcius = new double[] {};

    inputs.intakePositionRad = Rotation2d.fromRadians(intakeSim.getAngleRads());
    inputs.intakeVelocityRadPerSec = intakeSim.getVelocityRadPerSec();
    inputs.intakeAppliedVolts = intakeAppliedVolts;
    inputs.intakeCurrentAmps = new double[] {Math.abs(intakeSim.getCurrentDrawAmps())};
    inputs.intakeTempCelcius = new double[] {};
  }

  @Override
  public void setRollersVoltage(double volts) {
    rollersAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    rollersSim.setInputVoltage(rollersAppliedVolts);
  }

  @Override
  public void setIntakeVoltage(double volts) {
    intakeAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    intakeSim.setInputVoltage(intakeAppliedVolts);
  }
}
