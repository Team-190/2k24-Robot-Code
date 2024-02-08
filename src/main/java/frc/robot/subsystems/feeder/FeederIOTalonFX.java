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
  private final TalonFX lowerLeftFeederTalon;
  private final TalonFX lowerRightFeederTalon;

  private final StatusSignal<Double> upperPosition;
  private final StatusSignal<Double> upperVelocity;
  private final StatusSignal<Double> upperAppliedVolts;
  private final StatusSignal<Double> upperCurrent;
  private final StatusSignal<Double> upperTemperature;

  private final StatusSignal<Double> lowerLeftPosition;
  private final StatusSignal<Double> lowerLeftVelocity;
  private final StatusSignal<Double> lowerLeftAppliedVolts;
  private final StatusSignal<Double> lowerLeftCurrent;
  private final StatusSignal<Double> lowerLeftTemperature;

  private final StatusSignal<Double> lowerRightPosition;
  private final StatusSignal<Double> lowerRightVelocity;
  private final StatusSignal<Double> lowerRightAppliedVolts;
  private final StatusSignal<Double> lowerRightCurrent;
  private final StatusSignal<Double> lowerRightTemperature;

  private final double UPPER_GEAR_RATIO = 2.0;
  private final double LOWER_GEAR_RATIO = 72.0 / 34.0;

  private final Alert upperDisconnectedAlert =
      new Alert("Feeder upper Talon is disconnected, check CAN bus.", AlertType.ERROR);

  private final Alert lowerLeftDisconnectedAlert =
      new Alert("Feeder lower left Talon is disconnected, check CAN bus.", AlertType.ERROR);
      
  private final Alert lowerRightDisconnectedAlert =
      new Alert("Feeder lower right Talon is disconnected, check CAN bus.", AlertType.ERROR);

  public FeederIOTalonFX() {
    switch (Constants.ROBOT) {
      case ROBOT_2K24_C:
        lowerLeftFeederTalon = new TalonFX(42);
        lowerRightFeederTalon = new TalonFX(47);
        upperFeederTalon = new TalonFX(43);
        break;
      case ROBOT_2K24_P:
        lowerLeftFeederTalon = new TalonFX(42);
                lowerRightFeederTalon = new TalonFX(47);

        upperFeederTalon = new TalonFX(43);
        break;
      case ROBOT_2K24_TEST:
        lowerLeftFeederTalon = new TalonFX(42);
                lowerRightFeederTalon = new TalonFX(47);

        upperFeederTalon = new TalonFX(43);
        break;
      default:
        throw new RuntimeException("Invalid robot");
    }

    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    upperFeederTalon.getConfigurator().apply(config);
    lowerLeftFeederTalon.getConfigurator().apply(config);
    lowerRightFeederTalon.getConfigurator().apply(config);

    lowerRightFeederTalon.setInverted(true);

    upperPosition = upperFeederTalon.getPosition();
    upperVelocity = upperFeederTalon.getVelocity();
    upperAppliedVolts = upperFeederTalon.getMotorVoltage();
    upperCurrent = upperFeederTalon.getSupplyCurrent();
    upperTemperature = upperFeederTalon.getDeviceTemp();

    lowerLeftPosition = lowerLeftFeederTalon.getPosition();
    lowerLeftVelocity = lowerLeftFeederTalon.getVelocity();
    lowerLeftAppliedVolts = lowerLeftFeederTalon.getMotorVoltage();
    lowerLeftCurrent = lowerLeftFeederTalon.getSupplyCurrent();
    lowerLeftTemperature = lowerLeftFeederTalon.getDeviceTemp();

    lowerRightPosition = lowerRightFeederTalon.getPosition();
    lowerRightVelocity = lowerRightFeederTalon.getVelocity();
    lowerRightAppliedVolts = lowerRightFeederTalon.getMotorVoltage();
    lowerRightCurrent = lowerRightFeederTalon.getSupplyCurrent();
    lowerRightTemperature = lowerRightFeederTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, upperVelocity, lowerLeftVelocity, lowerRightVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, upperPosition, upperAppliedVolts, upperCurrent, upperTemperature, lowerLeftPosition, lowerLeftAppliedVolts, lowerLeftCurrent, lowerLeftTemperature, lowerRightPosition, lowerRightAppliedVolts, lowerRightCurrent, lowerRightTemperature);
    upperFeederTalon.optimizeBusUtilization();
    lowerLeftFeederTalon.optimizeBusUtilization();
    lowerRightFeederTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    boolean upperConnected =
        BaseStatusSignal.refreshAll(
                upperVelocity, upperPosition, upperAppliedVolts, upperCurrent, upperTemperature)
            .isOK();
    upperDisconnectedAlert.set(!upperConnected);

    boolean lowerLeftConnected =
        BaseStatusSignal.refreshAll(
                lowerLeftVelocity, lowerLeftPosition, lowerLeftAppliedVolts, lowerLeftCurrent, lowerLeftTemperature)
            .isOK();
    lowerLeftDisconnectedAlert.set(!lowerLeftConnected);

    boolean lowerRightConnected =
        BaseStatusSignal.refreshAll(
                lowerRightVelocity, lowerRightPosition, lowerRightAppliedVolts, lowerRightCurrent, lowerRightTemperature)
            .isOK();
    lowerRightDisconnectedAlert.set(!lowerRightConnected);

    inputs.upperPositionRad =
        Units.rotationsToRadians(upperPosition.getValueAsDouble()) / UPPER_GEAR_RATIO;
    inputs.upperVelocityRadPerSec =
        Units.rotationsToRadians(upperVelocity.getValueAsDouble()) / UPPER_GEAR_RATIO;
    inputs.upperAppliedVolts = upperAppliedVolts.getValueAsDouble();
    inputs.upperCurrentAmps = new double[] {upperCurrent.getValueAsDouble()};
    inputs.upperTempCelcius = new double[] {upperTemperature.getValueAsDouble()};

    inputs.lowerLeftPositionRad =
        Units.rotationsToRadians(lowerLeftPosition.getValueAsDouble()) / LOWER_GEAR_RATIO;
    inputs.lowerLeftVelocityRadPerSec =
        Units.rotationsToRadians(lowerLeftVelocity.getValueAsDouble()) / LOWER_GEAR_RATIO;
    inputs.lowerLeftAppliedVolts = lowerLeftAppliedVolts.getValueAsDouble();
    inputs.lowerLeftCurrentAmps = new double[] {lowerLeftCurrent.getValueAsDouble()};
    inputs.lowerLeftTempCelcius = new double[] { lowerLeftTemperature.getValueAsDouble() };
    
    inputs.lowerRightPositionRad =
        Units.rotationsToRadians(lowerRightPosition.getValueAsDouble()) / LOWER_GEAR_RATIO;
    inputs.lowerRightVelocityRadPerSec =
        Units.rotationsToRadians(lowerRightVelocity.getValueAsDouble()) / LOWER_GEAR_RATIO;
    inputs.lowerRightAppliedVolts = lowerRightAppliedVolts.getValueAsDouble();
    inputs.lowerRightCurrentAmps = new double[] {lowerRightCurrent.getValueAsDouble()};
    inputs.lowerRightTempCelcius = new double[] {lowerRightTemperature.getValueAsDouble()};
  }

  @Override
  public void setUpperVoltage(double volts) {
    upperFeederTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setLowerVoltage(double volts) {
    lowerLeftFeederTalon.setControl(new VoltageOut(volts));
    lowerRightFeederTalon.setControl(new VoltageOut(volts));
  }
}
