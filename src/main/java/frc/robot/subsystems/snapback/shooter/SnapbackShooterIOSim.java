package frc.robot.subsystems.snapback.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class SnapbackShooterIOSim implements SnapbackShooterIO {

  private DCMotorSim leftFlywheelMotorSim;
  private DCMotorSim rightFlywheelMotorSim;
  private DCMotorSim acceleratorMotorSim;

  private double leftFlywheelMotorAppliedVolts;
  private double rightFlywheelMotorAppliedVolts;
  private double acceleratorMotorAppliedVolts;

  private ProfiledPIDController leftFlywheelPIDController;
  private ProfiledPIDController rightFlywheelPIDController;
  private SimpleMotorFeedforward leftFlywheelFeedforward;
  private SimpleMotorFeedforward rightFlywheelFeedforward;

  public SnapbackShooterIOSim() {
    leftFlywheelMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                SnapbackShooterConstants.LEFT_MOTOR_CONFIG,
                0.004,
                SnapbackShooterConstants.LEFT_GEAR_RATIO),
            SnapbackShooterConstants.LEFT_MOTOR_CONFIG);
    rightFlywheelMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                SnapbackShooterConstants.RIGHT_MOTOR_CONFIG,
                0.004,
                SnapbackShooterConstants.RIGHT_GEAR_RATIO),
            SnapbackShooterConstants.RIGHT_MOTOR_CONFIG);
    acceleratorMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                SnapbackShooterConstants.ACCELERATOR_MOTOR_CONFIG,
                0.004,
                SnapbackShooterConstants.ACCELERATOR_GEAR_RATIO),
            SnapbackShooterConstants.ACCELERATOR_MOTOR_CONFIG);

    leftFlywheelMotorAppliedVolts = 0.0;
    rightFlywheelMotorAppliedVolts = 0.0;
    acceleratorMotorAppliedVolts = 0.0;

    leftFlywheelPIDController =
        new ProfiledPIDController(
            SnapbackShooterConstants.GAINS.kp().get(),
            0.0,
            SnapbackShooterConstants.GAINS.kd().get(),
            new TrapezoidProfile.Constraints(
                SnapbackShooterConstants.CONSTRAINTS.maxAccelerationRadiansPerSecondSquared().get(),
                Double.POSITIVE_INFINITY));
    rightFlywheelPIDController =
        new ProfiledPIDController(
            SnapbackShooterConstants.GAINS.kp().get(),
            0.0,
            SnapbackShooterConstants.GAINS.kd().get(),
            new TrapezoidProfile.Constraints(
                SnapbackShooterConstants.CONSTRAINTS.maxAccelerationRadiansPerSecondSquared().get(),
                Double.POSITIVE_INFINITY));
    leftFlywheelPIDController.setTolerance(
        SnapbackShooterConstants.CONSTRAINTS.goalToleranceRadiansPerSecond().get());
    rightFlywheelPIDController.setTolerance(
        SnapbackShooterConstants.CONSTRAINTS.goalToleranceRadiansPerSecond().get());

    leftFlywheelFeedforward =
        new SimpleMotorFeedforward(
            SnapbackShooterConstants.GAINS.ks().get(),
            SnapbackShooterConstants.GAINS.kv().get(),
            SnapbackShooterConstants.GAINS.ka().get());
    rightFlywheelFeedforward =
        new SimpleMotorFeedforward(
            SnapbackShooterConstants.GAINS.ks().get(),
            SnapbackShooterConstants.GAINS.kv().get(),
            SnapbackShooterConstants.GAINS.ka().get());
  }

  @Override
  public void updateInputs(SnapbackShooterIOInputs inputs) {

    leftFlywheelMotorSim.update(Constants.LOOP_PERIOD_SECONDS);
    rightFlywheelMotorSim.update(Constants.LOOP_PERIOD_SECONDS);
    acceleratorMotorSim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.leftPosition = Rotation2d.fromRadians(leftFlywheelMotorSim.getAngularPositionRad());
    inputs.leftVelocityRadiansPerSecond = leftFlywheelMotorSim.getAngularVelocityRadPerSec();
    inputs.leftAppliedVolts = leftFlywheelMotorAppliedVolts;
    inputs.leftCurrentAmps = leftFlywheelMotorSim.getCurrentDrawAmps();
    inputs.leftVelocityGoalRadiansPerSecond = leftFlywheelPIDController.getSetpoint().velocity;

    inputs.leftVelocityErrorRadiansPerSecond = leftFlywheelPIDController.getVelocityError();

    inputs.rightPosition = Rotation2d.fromRadians(rightFlywheelMotorSim.getAngularPositionRad());
    inputs.rightVelocityRadiansPerSecond = rightFlywheelMotorSim.getAngularVelocityRadPerSec();
    inputs.rightAppliedVolts = rightFlywheelMotorAppliedVolts;
    inputs.rightCurrentAmps = rightFlywheelMotorSim.getCurrentDrawAmps();
    inputs.rightVelocityGoalRadiansPerSecond = rightFlywheelPIDController.getSetpoint().velocity;

    inputs.rightVelocityErrorRadiansPerSecond = rightFlywheelPIDController.getVelocityError();
    inputs.acceleratorPosition =
        Rotation2d.fromRadians(acceleratorMotorSim.getAngularPositionRad());
    inputs.acceleratorVelocityRadiansPerSecond = acceleratorMotorSim.getAngularVelocityRadPerSec();
    inputs.acceleratorAppliedVolts = acceleratorMotorAppliedVolts;
    inputs.acceleratorCurrentAmps = acceleratorMotorSim.getCurrentDrawAmps();
  }

  @Override
  public void setLeftVoltage(double volts) {
    leftFlywheelMotorAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    leftFlywheelMotorSim.setInputVoltage(leftFlywheelMotorAppliedVolts);
  }

  @Override
  public void setRightVoltage(double volts) {
    rightFlywheelMotorAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    rightFlywheelMotorSim.setInputVoltage(rightFlywheelMotorAppliedVolts);
  }

  @Override
  public void setAcceleratorVoltage(double volts) {
    acceleratorMotorAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    acceleratorMotorSim.setInputVoltage(acceleratorMotorAppliedVolts);
  }

  @Override
  public void setLeftVelocity(double velocityRadiansPerSecond) {
    leftFlywheelMotorSim.setInputVoltage(
        leftFlywheelPIDController.calculate(velocityRadiansPerSecond)
            + leftFlywheelFeedforward.calculate(leftFlywheelPIDController.getSetpoint().velocity));
  }

  @Override
  public void setRightVelocity(double velocityRadiansPerSecond) {
    rightFlywheelMotorSim.setInputVoltage(
        rightFlywheelPIDController.calculate(velocityRadiansPerSecond)
            + rightFlywheelFeedforward.calculate(
                rightFlywheelPIDController.getSetpoint().velocity));
  }

  @Override
  public void setPID(double kp, double ki, double kd) {
    leftFlywheelPIDController.setPID(kp, ki, kd);
    rightFlywheelPIDController.setPID(kp, ki, kd);
  }

  @Override
  public void setFeedforward(double ks, double kv, double ka) {
    leftFlywheelFeedforward = new SimpleMotorFeedforward(ks, kv, ka);
    rightFlywheelFeedforward = new SimpleMotorFeedforward(ks, kv, ka);
  }

  @Override
  public void setProfile(
      double maxAccelerationRadiansPerSecondSquared, double goalToleranceRadiansPerSecond) {
    leftFlywheelPIDController.setConstraints(
        new TrapezoidProfile.Constraints(
            maxAccelerationRadiansPerSecondSquared, Double.POSITIVE_INFINITY));
    rightFlywheelPIDController.setConstraints(
        new TrapezoidProfile.Constraints(
            maxAccelerationRadiansPerSecondSquared, Double.POSITIVE_INFINITY));
    leftFlywheelPIDController.setTolerance(goalToleranceRadiansPerSecond);
    rightFlywheelPIDController.setTolerance(goalToleranceRadiansPerSecond);
  }

  @Override
  public boolean atGoal() {
    return leftFlywheelPIDController.atGoal() && rightFlywheelPIDController.atGoal();
  }
}
