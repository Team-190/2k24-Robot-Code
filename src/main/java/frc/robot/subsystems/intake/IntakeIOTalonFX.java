package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX talon;

  private final StatusSignal<Double> position;
  private final StatusSignal<Double> velocity;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> current;
  private final StatusSignal<Double> temperature;

  private final double GEAR_RATIO = 1.6;

  private final Alert disconnectedAlert =
      new Alert("Intake Talon is disconnected, check CAN bus.", AlertType.ERROR);

  public IntakeIOTalonFX() {
    switch (Constants.ROBOT) {
      case ROBOT_2K24_C:
        talon = new TalonFX(40);
        break;
      case ROBOT_2K24_P:
        talon = new TalonFX(40);
        break;
      case ROBOT_2K24_TEST:
        talon = new TalonFX(40);
        break;
      default:
        throw new RuntimeException("Invalid robot");
    }

    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    talon.getConfigurator().apply(config);

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    current = talon.getSupplyCurrent();
    temperature = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, velocity);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, appliedVolts, current, temperature);
    talon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    boolean connected =
        BaseStatusSignal.refreshAll(velocity, position, appliedVolts, current, temperature).isOK();
    disconnectedAlert.set(!connected);

    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble()) / GEAR_RATIO;
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble()) / GEAR_RATIO;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {current.getValueAsDouble()};
    inputs.tempCelcius = new double[] {temperature.getValueAsDouble()};
  }

  @Override
  public void setVoltage(double volts) {
    talon.setControl(new VoltageOut(volts));
  }
}
