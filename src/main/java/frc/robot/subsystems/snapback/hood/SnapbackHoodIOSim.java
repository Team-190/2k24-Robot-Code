package frc.robot.subsystems.snapback.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class SnapbackHoodIOSim implements SnapbackHoodIO {
  private final DCMotorSim motorSim;
  private double appliedVolts = 0.0;

  private ProfiledPIDController controller;
  private SimpleMotorFeedforward feedforward;

  public SnapbackHoodIOSim() {
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                SnapbackHoodConstants.HOOD_GEARBOX, 0.004, SnapbackHoodConstants.GEAR_REDUCTION),
            SnapbackHoodConstants.HOOD_GEARBOX);
    controller =
        new ProfiledPIDController(
            SnapbackHoodConstants.GAINS.kp(),
            SnapbackHoodConstants.GAINS.ki(),
            SnapbackHoodConstants.GAINS.kd(),
            new TrapezoidProfile.Constraints(
                SnapbackHoodConstants.MAX_VELOCITY.get(),
                SnapbackHoodConstants.MAX_ACCELERATION.get()));
    controller.setTolerance(SnapbackHoodConstants.GOAL_TOLERANCE.get());

    feedforward =
        new SimpleMotorFeedforward(
            SnapbackHoodConstants.GAINS.ks(), SnapbackHoodConstants.GAINS.kv());
  }

  @Override
  public void updateInputs(SnapbackHoodIOInputs inputs) {
    motorSim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.position = Rotation2d.fromRadians(motorSim.getAngularPositionRad());
    inputs.velocityRadiansPerSecond = motorSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = motorSim.getCurrentDrawAmps();
    inputs.positionSetpoint = Rotation2d.fromRotations(controller.getSetpoint().position);
    inputs.positionError = Rotation2d.fromRotations(controller.getPositionError());
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    motorSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void setPositionGoal(Rotation2d position) {
    appliedVolts =
        controller.calculate(position.getRadians())
            + feedforward
                .calculate(RadiansPerSecond.of(controller.getSetpoint().velocity))
                .in(Volts);
    motorSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void setPosition(Rotation2d position) {
    motorSim.setAngle(position.getRadians());
  }

  @Override
  public boolean atGoal() {
    return controller.atGoal();
  }
}
