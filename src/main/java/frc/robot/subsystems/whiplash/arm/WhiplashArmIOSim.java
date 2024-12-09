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
            WhiplashArmConstants.ARM_MOTOR_CONFIG,
            WhiplashArmConstants.ARM_GEAR_RATIO,
            WhiplashArmConstants.ARM_MOMENT_OF_INERTIA,
            WhiplashArmConstants.ARM_LENGTH_METERS,
            WhiplashArmConstants.ARM_MIN_ANGLE,
            WhiplashArmConstants.ARM_MAX_ANGLE,
            true,
            WhiplashArmConstants.ARM_MIN_ANGLE);

    feedback =
        new ProfiledPIDController(
            WhiplashArmConstants.ARM_KP.get(),
            0.0,
            WhiplashArmConstants.ARM_KD.get(),
            new Constraints(
                WhiplashArmConstants.ARM_MAX_VELOCITY.get(),
                WhiplashArmConstants.ARM_MAX_ACCELERATION.get()));
    feedback.setTolerance(WhiplashArmConstants.GOAL_TOLERANCE.get());

    feedforward =
        new ArmFeedforward(
            WhiplashArmConstants.ARM_KS.get(),
            WhiplashArmConstants.ARM_KG.get(),
            WhiplashArmConstants.ARM_KV.get());

    AppliedVolts = 0.0;
  }

  @Override
  public void updateInputs(WhiplashArmIOInputs inputs) {
    sim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.armPosition = Rotation2d.fromRadians(sim.getAngleRads());
    inputs.armVelocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.armAppliedVolts = AppliedVolts;
    inputs.armCurrentAmps = sim.getCurrentDrawAmps();

    inputs.armAbsolutePosition = Rotation2d.fromRadians(sim.getAngleRads());

    inputs.positionSetpoint = Rotation2d.fromRadians(feedback.getSetpoint().position);
    inputs.positionError = Rotation2d.fromRadians(feedback.getPositionError());
    inputs.positionGoal = Rotation2d.fromRadians(feedback.getGoal().position);
  }

  @Override
  public void setArmVoltage(double volts) {
    AppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(AppliedVolts);
  }

  @Override
  public void stop() {
    AppliedVolts = 0.0;
    sim.setInputVoltage(0.0);
  }

  @Override
  public void setArmPosition(Rotation2d currentPosition, Rotation2d setpointPosition) {
    AppliedVolts =
        MathUtil.clamp(
            feedback.calculate(sim.getAngleRads(), setpointPosition.getRadians())
                + feedforward
                    .calculate(
                        Radians.of(sim.getAngleRads()),
                        RadiansPerSecond.of(sim.getVelocityRadPerSec()))
                    .in(Volts),
            -12.0,
            12.0);
    sim.setInputVoltage(AppliedVolts);
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
  public void setProfile(double max_velocity, double max_acceleration) {
    feedback.setConstraints(new Constraints(max_velocity, max_acceleration));
  }

  @Override
  public boolean atSetpoint() {
    return feedback.atSetpoint();
  }
}
