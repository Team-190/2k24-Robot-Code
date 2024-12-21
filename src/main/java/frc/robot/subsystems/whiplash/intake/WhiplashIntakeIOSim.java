package frc.robot.subsystems.whiplash.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class WhiplashIntakeIOSim implements WhiplashIntakeIO {
  private final DCMotorSim topMotorSim;
  private final DCMotorSim bottomMotorSim;
  private final DCMotorSim acceleratorMotorSim;

  private double topAppliedVolts;
  private double bottomAppliedVolts;
  private double acceleratorAppliedVolts;

  public WhiplashIntakeIOSim() {
    topMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                WhiplashIntakeConstants.TOP_MOTOR_CONFIG,
                0.004,
                WhiplashIntakeConstants.TOP_GEAR_RATIO),
            WhiplashIntakeConstants.TOP_MOTOR_CONFIG);
    bottomMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                WhiplashIntakeConstants.BOTTOM_MOTOR_CONFIG,
                0.004,
                WhiplashIntakeConstants.BOTTOM_GEAR_RATIO),
            WhiplashIntakeConstants.BOTTOM_MOTOR_CONFIG);
    acceleratorMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                WhiplashIntakeConstants.ACCELERATOR_MOTOR_CONFIG,
                0.004,
                WhiplashIntakeConstants.ACCELERATOR_GEAR_RATIO),
            WhiplashIntakeConstants.ACCELERATOR_MOTOR_CONFIG);

    topAppliedVolts = 0.0;
    bottomAppliedVolts = 0.0;
    acceleratorAppliedVolts = 0.0;
  }

  @Override
  public void updateInputs(WhiplashIntakeIOInputs inputs) {
    topMotorSim.setInputVoltage(MathUtil.clamp(topAppliedVolts, -12.0, 12.0));
    bottomMotorSim.setInputVoltage(MathUtil.clamp(bottomAppliedVolts, -12.0, 12.0));
    acceleratorMotorSim.setInputVoltage(MathUtil.clamp(acceleratorAppliedVolts, -12.0, 12.0));

    topMotorSim.update(Constants.LOOP_PERIOD_SECONDS);
    bottomMotorSim.update(Constants.LOOP_PERIOD_SECONDS);
    acceleratorMotorSim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.topPosition = Rotation2d.fromRadians(topMotorSim.getAngularPositionRad());
    inputs.topVelocityRadiansPerSecond = topMotorSim.getAngularVelocityRadPerSec();
    inputs.topAppliedVolts = topAppliedVolts;
    inputs.topCurrentAmps = topMotorSim.getCurrentDrawAmps();

    inputs.bottomPosition = Rotation2d.fromRadians(bottomMotorSim.getAngularPositionRad());
    inputs.bottomVelocityRadiansPerSecond = bottomMotorSim.getAngularVelocityRadPerSec();
    inputs.bottomAppliedVolts = bottomAppliedVolts;
    inputs.bottomCurrentAmps = bottomMotorSim.getCurrentDrawAmps();

    inputs.acceleratorPosition =
        Rotation2d.fromRadians(acceleratorMotorSim.getAngularPositionRad());
    inputs.acceleratorVelocityRadiansPerSecond = acceleratorMotorSim.getAngularVelocityRadPerSec();
    inputs.acceleratorAppliedVolts = acceleratorAppliedVolts;
    inputs.acceleratorCurrentAmps = acceleratorMotorSim.getCurrentDrawAmps();
  }

  @Override
  public void setTopVoltage(double volts) {
    topAppliedVolts = volts;
  }

  @Override
  public void setBottomVoltage(double volts) {
    bottomAppliedVolts = volts;
  }

  @Override
  public void setAcceleratorVoltage(double volts) {
    acceleratorAppliedVolts = volts;
  }

  @Override
  public void stop() {
    topAppliedVolts = 0.0;
    bottomAppliedVolts = 0.0;
    acceleratorAppliedVolts = 0.0;
  }
}
