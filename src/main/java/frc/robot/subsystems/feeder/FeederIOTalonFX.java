package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class FeederIOTalonFX implements FeederIO {
  private final TalonFX talon;

  private final StatusSignal<Double> position;
  private final StatusSignal<Double> velocity;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> current;
  private final StatusSignal<Double> temperature;

  private final double GEAR_RATIO = 1;

  public FeederIOTalonFX() {
    switch (Constants.ROBOT) {
      case ROBOT_2K24_C:
        talon = new TalonFX(10);
        break;
      case ROBOT_2K24_P:
        talon = new TalonFX(10);
        break;
      case ROBOT_2K24_TEST:
        talon = new TalonFX(10);
        break;
      default:
        throw new RuntimeException("Invalid robot");
    }

    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    talon.getConfigurator().apply(config);

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    current = talon.getStatorCurrent();
    temperature = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, velocity);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, appliedVolts, current, temperature);
    talon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocity, position, appliedVolts, current, temperature);

    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble()) * GEAR_RATIO;
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble()) * GEAR_RATIO;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {current.getValueAsDouble()};
    inputs.tempCelcius = new double[] {temperature.getValueAsDouble()};
  }

  @Override
  public void setVoltage(double volts) {
    talon.setControl(new VoltageOut(volts));
  }
}
