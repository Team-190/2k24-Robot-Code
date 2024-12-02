package frc.robot.subsystems.whiplash.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.Constants;

public class ShooterIOSim implements ShooterIO {

  private DCMotorSim bottomFlywheelMotorSim;
  private DCMotorSim topFlywheelMotorSim;
  private DCMotorSim acceleratorMotorSim;

  private double bottomFlywheelMotorAppliedVolts;
  private double topFlywheelMotorAppliedVolts;

  private ProfiledPIDController bottomFlywheelPIDController;
  private ProfiledPIDController topFlywheelPIDController;
  private SimpleMotorFeedforward bottomFlywheelFeedforward;
  private SimpleMotorFeedforward topFlywheelFeedforward;

  public ShooterIOSim() {
    bottomFlywheelMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ShooterConstants.FLYWHEEL_GEARBOX, 0.004, ShooterConstants.FLYWHEEL_GEAR_REDUCTION),
            ShooterConstants.FLYWHEEL_GEARBOX,
            0.004);
    topFlywheelMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ShooterConstants.FLYWHEEL_GEARBOX, 0.004, ShooterConstants.FLYWHEEL_GEAR_REDUCTION),
            ShooterConstants.FLYWHEEL_GEARBOX,
            0.004);

    bottomFlywheelMotorAppliedVolts = 0.0;
    topFlywheelMotorAppliedVolts = 0.0;

    bottomFlywheelPIDController =
        new ProfiledPIDController(
            ShooterConstants.GAINS.kp().get(),
            ShooterConstants.GAINS.ki().get(),
            ShooterConstants.GAINS.kd().get(),
            new TrapezoidProfile.Constraints(
                ShooterConstants.MAX_ACCELERATION.get(), Double.POSITIVE_INFINITY));
    topFlywheelPIDController =
        new ProfiledPIDController(
            ShooterConstants.GAINS.kp().get(),
            ShooterConstants.GAINS.ki().get(),
            ShooterConstants.GAINS.kd().get(),
            new TrapezoidProfile.Constraints(
                ShooterConstants.MAX_ACCELERATION.get(), Double.POSITIVE_INFINITY));
    bottomFlywheelPIDController.setTolerance(ShooterConstants.FLYWHEEL_TOLERANCE_RAD_PER_SEC);
    topFlywheelPIDController.setTolerance(ShooterConstants.FLYWHEEL_TOLERANCE_RAD_PER_SEC);

    bottomFlywheelFeedforward =
        new SimpleMotorFeedforward(
            ShooterConstants.GAINS.ks(), ShooterConstants.GAINS.kv(), ShooterConstants.GAINS.ka());
    topFlywheelFeedforward =
        new SimpleMotorFeedforward(
            ShooterConstants.GAINS.ks(), ShooterConstants.GAINS.kv(), ShooterConstants.GAINS.ka());
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    bottomFlywheelMotorSim.update(Constants.LOOP_PERIOD_SECONDS);
    topFlywheelMotorSim.update(Constants.LOOP_PERIOD_SECONDS);
    acceleratorMotorSim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.bottomPosition = Rotation2d.fromRadians(bottomFlywheelMotorSim.getAngularPositionRad());
    inputs.bottomVelocityRadiansPerSecond = bottomFlywheelMotorSim.getAngularVelocityRadPerSec();
    inputs.bottomAppliedVolts = bottomFlywheelMotorAppliedVolts;
    inputs.bottomCurrentAmps = bottomFlywheelMotorSim.getCurrentDrawAmps();
    inputs.bottomVelocityGoalRadiansPerSecond = bottomFlywheelPIDController.getSetpoint().velocity;

    inputs.bottomVelocityErrorRadiansPerSecond = bottomFlywheelPIDController.getVelocityError();

    inputs.topPosition = Rotation2d.fromRadians(topFlywheelMotorSim.getAngularPositionRad());
    inputs.topVelocityRadiansPerSecond = topFlywheelMotorSim.getAngularVelocityRadPerSec();
    inputs.topAppliedVolts = topFlywheelMotorAppliedVolts;
    inputs.topCurrentAmps = topFlywheelMotorSim.getCurrentDrawAmps();
    inputs.topVelocityGoalRadiansPerSecond = topFlywheelPIDController.getSetpoint().velocity;

    inputs.topVelocityErrorRadiansPerSecond = topFlywheelPIDController.getVelocityError();
  }

  @Override
  public void setBottomVoltage(double volts) {
    bottomFlywheelMotorAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    bottomFlywheelMotorSim.setInputVoltage(bottomFlywheelMotorAppliedVolts);
  }

  @Override
  public void setTopVoltage(double volts) {
    topFlywheelMotorAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    topFlywheelMotorSim.setInputVoltage(topFlywheelMotorAppliedVolts);
  }

  @Override
  public void setBottomVelocityGoal(double velocityRadiansPerSecond) {
    bottomFlywheelMotorSim.setInputVoltage(
        bottomFlywheelPIDController.calculate(velocityRadiansPerSecond)
            + bottomFlywheelFeedforward
                .calculate(RadiansPerSecond.of(bottomFlywheelPIDController.getSetpoint().velocity))
                .in(Volts));
  }

  @Override
  public void setTopVelocityGoal(double velocityRadiansPerSecond) {
    topFlywheelMotorSim.setInputVoltage(
        topFlywheelPIDController.calculate(velocityRadiansPerSecond)
            + topFlywheelFeedforward
                .calculate(RadiansPerSecond.of(topFlywheelPIDController.getSetpoint().velocity))
                .in(Volts));
  }

  @Override
  public boolean atGoal() {
    return bottomFlywheelPIDController.atGoal() && topFlywheelPIDController.atGoal();
  }
}
