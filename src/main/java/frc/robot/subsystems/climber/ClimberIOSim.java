package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ClimberIOSim implements ClimberIO {
  private ElevatorSim leftClimberSim =
      new ElevatorSim(
          DCMotor.getKrakenX60(1), 10.0, 7.0, Units.inchesToMeters(0.25), 0.0, 0.0, false, 0.0);

  private ElevatorSim rightClimberSim =
      new ElevatorSim(
          DCMotor.getKrakenX60(1), 10.0, 7.0, Units.inchesToMeters(0.25), 0.0, 0.0, false, 0.0);

  private double leftAppliedVolts;
  private double rightAppliedVolts;

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.leftPositionMeters = leftClimberSim.getPositionMeters();
    inputs.leftVelocityMetersPerSec = leftClimberSim.getVelocityMetersPerSecond();
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftCurrentAmps = new double[] {leftClimberSim.getCurrentDrawAmps()};
    inputs.leftTempCelcius = new double[] {};

    inputs.rightPositionMeters = rightClimberSim.getPositionMeters();
    inputs.rightVelocityMetersPerSec = rightClimberSim.getVelocityMetersPerSecond();
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightCurrentAmps = new double[] {rightClimberSim.getCurrentDrawAmps()};
    inputs.rightTempCelcius = new double[] {};
  }

  @Override
  public void setLeftVoltage(double volts) {
    leftAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    leftClimberSim.setInputVoltage(leftAppliedVolts);
  }

  @Override
  public void setRightVoltage(double volts) {
    rightAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    rightClimberSim.setInputVoltage(leftAppliedVolts);
  }
}
