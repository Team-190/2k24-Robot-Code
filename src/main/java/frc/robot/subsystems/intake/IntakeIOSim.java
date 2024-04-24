package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {
  private DCMotorSim motorSim = new DCMotorSim(DCMotor.getKrakenX60(1), 2.0, 0.004);

  private double rollersAppliedVolts = 0.0;
  private boolean leftPosition = false;
  private boolean rightPosition = false;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    motorSim.update(Constants.LOOP_PERIOD_SECS);

    inputs.rollersPositionRad = motorSim.getAngularPositionRad();
    inputs.rollersVelocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
    inputs.rollersAppliedVolts = rollersAppliedVolts;
    inputs.rollersCurrentAmps = new double[] {Math.abs(motorSim.getCurrentDrawAmps())};
    inputs.rollersTempCelcius = new double[] {};

    inputs.leftPosition = leftPosition;
    inputs.rightPosition = rightPosition;
  }

  @Override
  public void setRollersVoltage(double volts) {
    rollersAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    motorSim.setInputVoltage(rollersAppliedVolts);
  }

  @Override
  public void setIntakePosition(boolean position) {
    leftPosition = !position;
    rightPosition = !position;
  }
}
