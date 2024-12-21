package frc.robot.subsystems.snapback.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class SnapbackHoodIOSim implements SnapbackHoodIO {
  private final DCMotorSim motorSim;

  private ProfiledPIDController feedback;
  private SimpleMotorFeedforward feedforward;

  private Rotation2d positionGoal = new Rotation2d();
  private double appliedVolts = 0.0;

  public SnapbackHoodIOSim() {
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                SnapbackHoodConstants.MOTOR_CONFIG, 0.004, SnapbackHoodConstants.GEAR_RATIO),
            SnapbackHoodConstants.MOTOR_CONFIG);

    feedback =
        new ProfiledPIDController(
            SnapbackHoodConstants.GAINS.kp().get(),
            0.0,
            SnapbackHoodConstants.GAINS.kd().get(),
            new TrapezoidProfile.Constraints(
                SnapbackHoodConstants.CONSTRAINTS.maxVelocityRadiansPerSecond().get(),
                SnapbackHoodConstants.CONSTRAINTS.maxAccelerationRadiansPerSecondSqaured().get()));
    feedback.setTolerance(SnapbackHoodConstants.CONSTRAINTS.goalToleranceRadians().get());
    feedforward =
        new SimpleMotorFeedforward(
            SnapbackHoodConstants.GAINS.ks().get(), SnapbackHoodConstants.GAINS.kv().get());
  }

  @Override
  public void updateInputs(SnapbackHoodIOInputs inputs) {
    motorSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    motorSim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.position = Rotation2d.fromRadians(motorSim.getAngularPositionRad());
    inputs.velocityRadiansPerSecond = motorSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = motorSim.getCurrentDrawAmps();
    inputs.positionGoal = positionGoal;
    inputs.positionSetpoint = Rotation2d.fromRotations(feedback.getSetpoint().position);
    inputs.positionError = Rotation2d.fromRotations(feedback.getPositionError());
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }

  @Override
  public void setPosition(Rotation2d position) {
    positionGoal = position;
    appliedVolts =
        feedback.calculate(position.getRadians())
            + feedforward.calculate(feedback.getSetpoint().velocity);
  }

  @Override
  public void setPID(double kp, double ki, double kd) {
    feedback.setPID(kp, ki, kd);
  }

  @Override
  public void setFeedforward(double ks, double kv, double ka) {
    feedforward = new SimpleMotorFeedforward(ks, kv, ka);
  }

  @Override
  public void setProfile(
      double maxVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared,
      double goalToleranceRadians) {
    feedback.setConstraints(
        new Constraints(maxVelocityRadiansPerSecond, maxAccelerationRadiansPerSecondSquared));
    feedback.setTolerance(goalToleranceRadians);
  }

  @Override
  public boolean atGoal() {
    return feedback.atGoal();
  }
}
