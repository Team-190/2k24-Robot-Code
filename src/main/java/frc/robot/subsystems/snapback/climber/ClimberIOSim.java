package frc.robot.subsystems.snapback.climber;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.Constants;

public class ClimberIOSim implements ClimberIO {
  private final DCMotorSim leftClimberMotorSim;
  private final DCMotorSim rightClimberMotorSim;

  public ClimberIOSim() {
    leftClimberMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ClimberConstants.CLIMBER_GEARBOX, 0.004, ClimberConstants.CLIMBER_GEAR_REDUCTION),
            ClimberConstants.CLIMBER_GEARBOX,
            0.004);
    rightClimberMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ClimberConstants.CLIMBER_GEARBOX, 0.004, ClimberConstants.CLIMBER_GEAR_REDUCTION),
            ClimberConstants.CLIMBER_GEARBOX,
            0.004);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {

    leftClimberMotorSim.update(Constants.LOOP_PERIOD_SECONDS);
    rightClimberMotorSim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.leftPositionMeters =
        leftClimberMotorSim.getAngularPositionRad() * ClimberConstants.CLIMBER_PULLY_RADIUS_METERS;
    inputs.leftVelocityMetersPerSecond = leftClimberMotorSim.getAngularVelocityRadPerSec();
    inputs.leftAppliedVolts = leftClimberMotorSim.getInputVoltage();
    inputs.leftCurrentAmps = leftClimberMotorSim.getCurrentDrawAmps();

    inputs.rightPositionMeters =
        rightClimberMotorSim.getAngularPositionRad() * ClimberConstants.CLIMBER_PULLY_RADIUS_METERS;
    inputs.rightVelocityMetersPerSecond = rightClimberMotorSim.getAngularVelocityRadPerSec();
    inputs.rightAppliedVolts = rightClimberMotorSim.getInputVoltage();
    inputs.rightCurrentAmps = rightClimberMotorSim.getCurrentDrawAmps();
  }

  @Override
  public void setLeftVoltage(double volts) {
    leftClimberMotorSim.setInputVoltage(volts);
  }

  @Override
  public void setRightVoltage(double volts) {
    rightClimberMotorSim.setInputVoltage(volts);
  }
}
