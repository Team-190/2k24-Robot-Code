package frc.robot.subsystems.whiplash.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.util.LinearProfile;

public class WhiplashShooterIOSim implements WhiplashShooterIO {

  private DCMotorSim topMotorSim;
  private DCMotorSim bottomMotorSim;

  private final PIDController feedback;
  private final LinearProfile profile;
  private SimpleMotorFeedforward feedforward;

  private double topAppliedVolts;
  private double bottomAppliedVolts;

  public WhiplashShooterIOSim() {
    topMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                WhiplashShooterConstants.TOP_MOTOR_CONFIG, 0.004, 1.0),
            WhiplashShooterConstants.TOP_MOTOR_CONFIG);
    bottomMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                WhiplashShooterConstants.BOTTOM_MOTOR_CONFIG, 0.004, 1.0),
            WhiplashShooterConstants.BOTTOM_MOTOR_CONFIG);

    feedback =
        new PIDController(
            WhiplashShooterConstants.GAINS.kp().get(),
            0.0,
            WhiplashShooterConstants.GAINS.kd().get());
    feedback.setTolerance(
        WhiplashShooterConstants.CONSTRAINTS.goalToleranceRadiansPerSecond().get());
    profile =
        new LinearProfile(
            WhiplashShooterConstants.CONSTRAINTS.maxAccelerationRadiansPerSecondSquared().get(),
            Constants.LOOP_PERIOD_SECONDS);
    feedforward =
        new SimpleMotorFeedforward(
            WhiplashShooterConstants.GAINS.ks().get(), WhiplashShooterConstants.GAINS.kv().get());

    topAppliedVolts = 0.0;
    bottomAppliedVolts = 0.0;
  }

  @Override
  public void updateInputs(WhiplashShooterIOInputs inputs) {
    topMotorSim.setInputVoltage(MathUtil.clamp(topAppliedVolts, -12.0, 12.0));
    bottomMotorSim.setInputVoltage(MathUtil.clamp(bottomAppliedVolts, -12.0, 12.0));
    topMotorSim.update(Constants.LOOP_PERIOD_SECONDS);
    bottomMotorSim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.topPosition = Rotation2d.fromRadians(topMotorSim.getAngularPositionRad());
    inputs.topVelocityRadiansPerSecond = topMotorSim.getAngularVelocityRadPerSec();
    inputs.topAppliedVolts = topAppliedVolts;
    inputs.topCurrentAmps = topMotorSim.getCurrentDrawAmps();
    inputs.topVelocityGoalRadiansPerSecond = profile.getGoal();
    inputs.topVelocitySetpointRadiansPerSecond = feedback.getSetpoint();
    inputs.topVelocityErrorRadiansPerSecond = feedback.getError();

    inputs.bottomPosition = Rotation2d.fromRadians(bottomMotorSim.getAngularPositionRad());
    inputs.bottomVelocityRadiansPerSecond = bottomMotorSim.getAngularVelocityRadPerSec();
    inputs.bottomAppliedVolts = bottomAppliedVolts;
    inputs.bottomCurrentAmps = bottomMotorSim.getCurrentDrawAmps();
    inputs.bottomVelocityGoalRadiansPerSecond = profile.getGoal();
    inputs.bottomVelocitySetpointRadiansPerSecond = feedback.getSetpoint();
    inputs.bottomVelocityErrorRadiansPerSecond = feedback.getError();
  }

  @Override
  public void setVoltage(double volts) {
    topAppliedVolts = volts;
    bottomAppliedVolts = volts;
  }

  @Override
  public void setTopVelocity(double velocityRadiansPerSecond) {
    profile.setGoal(velocityRadiansPerSecond, topMotorSim.getAngularVelocityRadPerSec());
    topAppliedVolts =
        feedback.calculate(topMotorSim.getAngularVelocityRadPerSec(), profile.calculateSetpoint())
            + feedforward.calculate(feedback.getSetpoint());
  }

  @Override
  public void setBottomVelocity(double velocityRadiansPerSecond) {
    profile.setGoal(velocityRadiansPerSecond, bottomMotorSim.getAngularVelocityRadPerSec());
    bottomAppliedVolts =
        feedback.calculate(
                bottomMotorSim.getAngularVelocityRadPerSec(), profile.calculateSetpoint())
            + feedforward.calculate(feedback.getSetpoint());
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
      double maxAccelerationRadiansPerSecondSquared, double goalToleranceRadiansPerSecond) {
    profile.setMaxAcceleration(maxAccelerationRadiansPerSecondSquared);
    feedback.setTolerance(goalToleranceRadiansPerSecond);
  }

  @Override
  public boolean atGoal() {
    return (Math.abs(profile.getGoal() - feedback.getSetpoint())
            <= WhiplashShooterConstants.CONSTRAINTS.goalToleranceRadiansPerSecond().get())
        && feedback.atSetpoint();
  }

  @Override
  public void stop() {
    topAppliedVolts = 0.0;
    bottomAppliedVolts = 0.0;
  }
}
