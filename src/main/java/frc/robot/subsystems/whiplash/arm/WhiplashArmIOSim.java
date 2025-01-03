package frc.robot.subsystems.whiplash.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class WhiplashArmIOSim implements WhiplashArmIO {
  private final SingleJointedArmSim sim;

  private final ProfiledPIDController feedback;
  private ArmFeedforward feedforward;

  private double AppliedVolts;

  public WhiplashArmIOSim() {
    sim =
        new SingleJointedArmSim(
            WhiplashArmConstants.MOTOR_CONFIG,
            WhiplashArmConstants.GEAR_RATIO,
            WhiplashArmConstants.MOMENT_OF_INERTIA,
            WhiplashArmConstants.LENGTH_METERS,
            WhiplashArmConstants.MIN_ANGLE,
            WhiplashArmConstants.MAX_ANGLE,
            true,
            WhiplashArmConstants.MIN_ANGLE);

    feedback =
        new ProfiledPIDController(
            WhiplashArmConstants.GAINS.kp().get(),
            0.0,
            WhiplashArmConstants.GAINS.kd().get(),
            new Constraints(
                WhiplashArmConstants.CONSTRAINTS.maxVelocityRadiansPerSecond().get(),
                WhiplashArmConstants.CONSTRAINTS.maxAccelerationRadiansPerSecondSqaured().get()));
    feedback.setTolerance(WhiplashArmConstants.CONSTRAINTS.goalToleranceRadians().get());
    feedforward =
        new ArmFeedforward(
            WhiplashArmConstants.GAINS.ks().get(),
            WhiplashArmConstants.GAINS.kg().get(),
            WhiplashArmConstants.GAINS.kv().get());

    AppliedVolts = 0.0;
  }

  @Override
  public void updateInputs(WhiplashArmIOInputs inputs) {
    sim.setInputVoltage(MathUtil.clamp(AppliedVolts, -12.0, 12.0));
    sim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.absolutePosition = Rotation2d.fromRadians(sim.getAngleRads());
    inputs.position = Rotation2d.fromRadians(sim.getAngleRads());
    inputs.velocityRadiansPerSecond = sim.getVelocityRadPerSec();
    inputs.appliedVolts = AppliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
    inputs.positionGoal = Rotation2d.fromRadians(feedback.getGoal().position);
    inputs.positionSetpoint = Rotation2d.fromRadians(feedback.getSetpoint().position);
    inputs.positionError = Rotation2d.fromRadians(feedback.getPositionError());
  }

  @Override
  public void setVoltage(double volts) {
    AppliedVolts = volts;
  }

  @Override
  public void setPosition(Rotation2d positionGoal) {
    AppliedVolts =
        feedback.calculate(sim.getAngleRads(), positionGoal.getRadians())
            + feedforward
                .calculate(
                    Radians.of(sim.getAngleRads()), RadiansPerSecond.of(sim.getVelocityRadPerSec()))
                .in(Volts);
  }

  @Override
  public void setPID(double kp, double ki, double kd) {
    feedback.setPID(kp, ki, kd);
  }

  @Override
  public void setFeedforward(double ks, double kg, double kv) {
    feedforward = new ArmFeedforward(ks, kg, kv);
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

  @Override
  public void stop() {
    AppliedVolts = 0.0;
    sim.setInputVoltage(0.0);
  }
}
