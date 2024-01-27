package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class PivotIOTalonFX implements PivotIO {
  private final TalonFX pivotTalon;

  private final StatusSignal<Double> position;
  private final StatusSignal<Double> velocity;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> currentAmps;
  private final StatusSignal<Double> tempCelcius;

  private final double GEAR_RATIO = 1.0;

  public PivotIOTalonFX() {
    switch (Constants.ROBOT) {
      case ROBOT_2K24_C:
        pivotTalon = new TalonFX(10);
        break;
      case ROBOT_2K24_P:
        pivotTalon = new TalonFX(10);
        break;
      case ROBOT_2K24_TEST:
        pivotTalon = new TalonFX(10);
        break;
      default:
        throw new RuntimeException("Invalid robot");
    }

    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotTalon.getConfigurator().apply(config);

    position = pivotTalon.getPosition();
    velocity = pivotTalon.getVelocity();
    appliedVolts = pivotTalon.getMotorVoltage();
    currentAmps = pivotTalon.getStatorCurrent();
    tempCelcius = pivotTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, currentAmps, tempCelcius);
    pivotTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, currentAmps, tempCelcius);

    inputs.position = Rotation2d.fromRotations(position.getValueAsDouble() * GEAR_RATIO);
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble() * GEAR_RATIO);
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {currentAmps.getValueAsDouble()};
    inputs.tempCelcius = new double[] {tempCelcius.getValueAsDouble()};
  }

  @Override
  public void setVoltage(double volts) {
    pivotTalon.setControl(new VoltageOut(volts));
  }
}
