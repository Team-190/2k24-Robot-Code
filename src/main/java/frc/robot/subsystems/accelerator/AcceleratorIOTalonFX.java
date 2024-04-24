package frc.robot.subsystems.accelerator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class AcceleratorIOTalonFX implements AcceleratorIO {
  private final TalonFX acceleratorTalon;

  private final StatusSignal<Double> position;
  private final StatusSignal<Double> velocity;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> current;
  private final StatusSignal<Double> temperature;

  private final double GEAR_RATIO = 2.0;

  private final Alert disconnectedAlert =
      new Alert("Accelerator Talon is disconnected, check CAN bus.", AlertType.ERROR);

  public AcceleratorIOTalonFX() {
    switch (Constants.ROBOT) {
      case SNAPBACK:
        acceleratorTalon = new TalonFX(15);
        break;
      case ROBOT_2K24_TEST:
        acceleratorTalon = new TalonFX(15);
        break;
      default:
        throw new RuntimeException("Invalid robot");
    }

    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Audio.AllowMusicDurDisable = true;
    config.Audio.BeepOnBoot = false;
    config.Audio.BeepOnConfig = false;

    position = acceleratorTalon.getPosition();
    velocity = acceleratorTalon.getVelocity();
    appliedVolts = acceleratorTalon.getMotorVoltage();
    current = acceleratorTalon.getSupplyCurrent();
    temperature = acceleratorTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, velocity);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, appliedVolts, current, temperature);
    acceleratorTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(AcceleratorIOInputs inputs) {
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
    acceleratorTalon.setControl(new VoltageOut(volts));
  }
}
