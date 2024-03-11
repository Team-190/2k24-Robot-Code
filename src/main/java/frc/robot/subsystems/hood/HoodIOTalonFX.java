package frc.robot.subsystems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class HoodIOTalonFX implements HoodIO {
  private final TalonFX hoodTalon;

  private final StatusSignal<Double> position;
  private final StatusSignal<Double> velocity;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> currentAmps;
  private final StatusSignal<Double> tempCelcius;

  private final double GEAR_RATIO = 5.0;

  private final Alert disconnecctedAlert =
      new Alert("Hood Talon is disconnected, check CAN bus.", AlertType.ERROR);

  public HoodIOTalonFX() {
    switch (Constants.ROBOT) {
      case SNAPBACK:
        hoodTalon = new TalonFX(7);
        break;
      case ROBOT_2K24_TEST:
        hoodTalon = new TalonFX(7);
        break;
      default:
        throw new RuntimeException("Invalid robot");
    }

    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodTalon.getConfigurator().apply(config);
    hoodTalon.setPosition(0.0);

    position = hoodTalon.getPosition();
    velocity = hoodTalon.getVelocity();
    appliedVolts = hoodTalon.getMotorVoltage();
    currentAmps = hoodTalon.getSupplyCurrent();
    tempCelcius = hoodTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, currentAmps, tempCelcius);
    hoodTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    boolean isConnected =
        BaseStatusSignal.refreshAll(position, velocity, appliedVolts, currentAmps, tempCelcius)
            .isOK();
    disconnecctedAlert.set(!isConnected);

    inputs.position = Rotation2d.fromRotations(position.getValueAsDouble() / GEAR_RATIO);
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble() / GEAR_RATIO);
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {currentAmps.getValueAsDouble()};
    inputs.tempCelcius = new double[] {tempCelcius.getValueAsDouble()};
  }

  @Override
  public void setVoltage(double volts) {
    hoodTalon.setControl(new VoltageOut(volts));
  }
}
