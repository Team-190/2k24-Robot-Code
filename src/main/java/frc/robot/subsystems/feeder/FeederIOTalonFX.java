package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class FeederIOTalonFX implements FeederIO {
  private final TalonFX upperFeederTalon;
  private final TalonFX lowerFeederTalon;

  private final StatusSignal<Double> upperPosition;
  private final StatusSignal<Double> upperVelocity;
  private final StatusSignal<Double> upperAppliedVolts;
  private final StatusSignal<Double> upperCurrent;
  private final StatusSignal<Double> upperTemperature;

  private final StatusSignal<Double> lowerPosition;
  private final StatusSignal<Double> lowerVelocity;
  private final StatusSignal<Double> lowerAppliedVolts;
  private final StatusSignal<Double> lowerCurrent;
  private final StatusSignal<Double> lowerTemperature;

  private final double GEAR_RATIO = 1;

  private final Alert disconnectedAlert =
      new Alert("Feeder Talon is disconnected, check CAN bus.", AlertType.ERROR);

  public FeederIOTalonFX() {
    switch (Constants.ROBOT) {
      case ROBOT_2K24_C:
        lowerFeederTalon = new TalonFX(42);
        upperFeederTalon = new TalonFX(43);
        break;
      case ROBOT_2K24_P:
        lowerFeederTalon = new TalonFX(42);
        upperFeederTalon = new TalonFX(43);
        break;
      case ROBOT_2K24_TEST:
        lowerFeederTalon = new TalonFX(42);
        upperFeederTalon = new TalonFX(43);
        break;
      default:
        throw new RuntimeException("Invalid robot");
    }

    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    upperFeederTalon.getConfigurator().apply(config);
    lowerFeederTalon.getConfigurator().apply(config);

    upperPosition = upperFeederTalon.getPosition();
    upperVelocity = upperFeederTalon.getVelocity();
    upperAppliedVolts = upperFeederTalon.getMotorVoltage();
    upperCurrent = upperFeederTalon.getStatorCurrent();
    upperTemperature = upperFeederTalon.getDeviceTemp();

    lowerPosition = lowerFeederTalon.getPosition();
    lowerVelocity = lowerFeederTalon.getVelocity();
    lowerAppliedVolts = lowerFeederTalon.getMotorVoltage();
    lowerCurrent = lowerFeederTalon.getStatorCurrent();
    lowerTemperature = lowerFeederTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, upperVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, upperPosition, upperAppliedVolts, upperCurrent, upperTemperature);
    upperFeederTalon.optimizeBusUtilization();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, lowerVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, lowerPosition, lowerAppliedVolts, lowerCurrent, lowerTemperature);
    lowerFeederTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    boolean connected =
        BaseStatusSignal.refreshAll(
                upperVelocity, upperPosition, upperAppliedVolts, upperCurrent, upperTemperature)
            .isOK();
    disconnectedAlert.set(!connected);

    inputs.upperPositionRad =
        Units.rotationsToRadians(upperPosition.getValueAsDouble()) * GEAR_RATIO;
    inputs.upperVelocityRadPerSec =
        Units.rotationsToRadians(upperVelocity.getValueAsDouble()) * GEAR_RATIO;
    inputs.upperAppliedVolts = upperAppliedVolts.getValueAsDouble();
    inputs.upperCurrentAmps = new double[] {upperCurrent.getValueAsDouble()};
    inputs.upperTempCelcius = new double[] {upperTemperature.getValueAsDouble()};

    inputs.lowerPositionRad =
        Units.rotationsToRadians(lowerPosition.getValueAsDouble()) * GEAR_RATIO;
    inputs.lowerVelocityRadPerSec =
        Units.rotationsToRadians(lowerVelocity.getValueAsDouble()) * GEAR_RATIO;
    inputs.lowerAppliedVolts = lowerAppliedVolts.getValueAsDouble();
    inputs.lowerCurrentAmps = new double[] {lowerCurrent.getValueAsDouble()};
    inputs.lowerTempCelcius = new double[] {lowerTemperature.getValueAsDouble()};
  }

  @Override
  public void setVoltage(double volts) {
    upperFeederTalon.setControl(new VoltageOut(volts));
    lowerFeederTalon.setControl(new VoltageOut(volts));
  }
}
