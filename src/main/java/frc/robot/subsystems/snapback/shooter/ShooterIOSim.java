package frc.robot.subsystems.snapback.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.Constants;

public class ShooterIOSim implements ShooterIO {

  private DCMotorSim leftFlywheelMotorSim;
  private DCMotorSim rightFlywheelMotorSim;
  private DCMotorSim acceleratorMotorSim;

  private double leftFlywheelMotorAppliedVolts = 0.0;
  private double rightFlywheelMotorAppliedVolts = 0.0;
  private double acceleratorMotorAppliedVolts = 0.0;

  private double leftFlywheelVelocitySetpoint = 0.0;
  private double rightFlywheelVelocitySetpoint = 0.0;

  public ShooterIOSim() {
    leftFlywheelMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ShooterConstants.FLYWHEEL_GEARBOX, 0.004, ShooterConstants.FLYWHEEL_GEAR_REDUCTION),
            ShooterConstants.FLYWHEEL_GEARBOX,
            0.004);
    rightFlywheelMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ShooterConstants.FLYWHEEL_GEARBOX, 0.004, ShooterConstants.FLYWHEEL_GEAR_REDUCTION),
            ShooterConstants.FLYWHEEL_GEARBOX,
            0.004);
    acceleratorMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ShooterConstants.ACCELERATOR_GEARBOX, 0.004, ShooterConstants.ACCELERATOR_GEAR_REDUCTION),
            ShooterConstants.ACCELERATOR_GEARBOX,
            0.004);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    leftFlywheelMotorSim.update(Constants.LOOP_PERIOD_SECONDS);
    rightFlywheelMotorSim.update(Constants.LOOP_PERIOD_SECONDS);
    acceleratorMotorSim.update(Constants.LOOP_PERIOD_SECONDS);

    leftFlywheelMotorSim.setInputVoltage(leftFlywheelMotorAppliedVolts);
    inputs.leftVelocitySetpointRadiansPerSecond = leftFlywheelVelocitySetpoint;
    inputs.leftVelocityErrorRadiansPerSecond =
        leftFlywheelVelocitySetpoint - leftFlywheelMotorSim.getAngularVelocityRadPerSec();
    inputs.leftPosition = Rotation2d.fromRadians(leftFlywheelMotorSim.getAngularPositionRad());
    inputs.leftVelocityRadiansPerSecond = leftFlywheelMotorSim.getAngularVelocityRadPerSec();
    inputs.leftAppliedVolts = leftFlywheelMotorAppliedVolts;
    inputs.leftCurrentAmps = leftFlywheelMotorSim.getCurrentDrawAmps();

    rightFlywheelMotorSim.setInputVoltage(rightFlywheelMotorAppliedVolts);
    inputs.rightVelocitySetpointRadiansPerSecond = rightFlywheelVelocitySetpoint;
    inputs.rightVelocityErrorRadiansPerSecond =
        rightFlywheelVelocitySetpoint - rightFlywheelMotorSim.getAngularVelocityRadPerSec();
    inputs.rightPosition = Rotation2d.fromRadians(rightFlywheelMotorSim.getAngularPositionRad());
    inputs.rightVelocityRadiansPerSecond = rightFlywheelMotorSim.getAngularVelocityRadPerSec();
    inputs.rightAppliedVolts = rightFlywheelMotorAppliedVolts;
    inputs.rightCurrentAmps = rightFlywheelMotorSim.getCurrentDrawAmps();

    acceleratorMotorSim.setInputVoltage(acceleratorMotorAppliedVolts);
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
  public void setLeftVelocityGoal(double velocityRadiansPerSecond) {
    leftFlywheelVelocitySetpoint = velocityRadiansPerSecond;
    leftFlywheelMotorSim.setAngularVelocity(leftFlywheelMotorAppliedVolts);
  }

  @Override
  public void setRightVelocitySetpoint(double velocityRadiansPerSecond) {
    rightFlywheelVelocitySetpoint = velocityRadiansPerSecond;
    rightFlywheelMotorSim.setAngularVelocity(rightFlywheelMotorAppliedVolts);
  }
}
