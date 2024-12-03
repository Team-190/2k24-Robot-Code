package frc.robot.subsystems.whiplash.climber;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.Constants;

public class ClimberIOSim implements ClimberIO {
  private final DCMotorSim climberMotorSim;

  public ClimberIOSim() {
    climberMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ClimberConstants.CLIMBER_GEARBOX, 0.004, ClimberConstants.CLIMBER_GEAR_REDUCTION),
            ClimberConstants.CLIMBER_GEARBOX,
            0.004);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {

    climberMotorSim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.climberPositionMeters =
        climberMotorSim.getAngularPositionRad() * ClimberConstants.CLIMBER_PULLEY_RADIUS_METERS;
    inputs.climberVelocityMetersPerSecond = climberMotorSim.getAngularVelocityRadPerSec();
    inputs.climberAppliedVolts = climberMotorSim.getInputVoltage();
    inputs.climberCurrentAmps = climberMotorSim.getCurrentDrawAmps();
  }

  @Override
  public void setClimberVoltage(double volts) {
    climberMotorSim.setInputVoltage(volts);
  }
}
