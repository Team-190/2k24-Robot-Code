package frc.robot.subsystems.snapback.hood;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.Constants;

public class HoodIOSim implements HoodIO {
  private final DCMotorSim motorSim;
  private double appliedVolts = 0.0;

  private ProfiledPIDController controller;
  private SimpleMotorFeedforward feedforward;

  public HoodIOSim() {
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                HoodConstants.HOOD_GEARBOX, 0.004, HoodConstants.GEAR_REDUCTION),
            HoodConstants.HOOD_GEARBOX,
            0.004);
    controller =
        new ProfiledPIDController(
            HoodConstants.GAINS.kp(),
            HoodConstants.GAINS.ki(),
            HoodConstants.GAINS.kd(),
            new TrapezoidProfile.Constraints(HoodConstants.GAINS.kv(), HoodConstants.GAINS.ka()));
    feedforward = new SimpleMotorFeedforward(HoodConstants.GAINS.ks(), HoodConstants.GAINS.kv());
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    motorSim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.position = Rotation2d.fromRadians(motorSim.getAngularPositionRad());
    inputs.velocityRadiansPerSecond = motorSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = motorSim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    motorSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void setPositionSetpoint(Rotation2d position) {
    appliedVolts =
        controller.calculate(position.getRadians())
            + feedforward.calculate(controller.getSetpoint().velocity);
    motorSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void setPosition(Rotation2d position) {
    motorSim.setAngle(position.getRadians());
  }
}
