package frc.robot.subsystems.whiplash.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

public class WhiplashIntakeIOTalonFX implements WhiplashIntakeIO {
  private final TalonFX topMotor;
  private final TalonFX bottomMotor;
  private final TalonFX acceleratorMotor;

  private final DigitalInput intakeSensor;
  private final DigitalInput middleSensor;
  private final DigitalInput finalSensor;

  private final StatusSignal<Angle> topPositionRotations;
  private final StatusSignal<AngularVelocity> topVelocityRotPerSec;
  private final StatusSignal<Voltage> topAppliedVolts;
  private final StatusSignal<Current> topCurrentAmps;
  private final StatusSignal<Temperature> topTemperatureCelsius;

  private final StatusSignal<Angle> bottomPositionRotations;
  private final StatusSignal<AngularVelocity> bottomVelocityRotPerSec;
  private final StatusSignal<Voltage> bottomAppliedVolts;
  private final StatusSignal<Current> bottomCurrentAmps;
  private final StatusSignal<Temperature> bottomTemperatureCelsius;

  private final StatusSignal<Angle> acceleratorPositionRotations;
  private final StatusSignal<AngularVelocity> acceleratorVelocityRotPerSec;
  private final StatusSignal<Voltage> acceleratorAppliedVolts;
  private final StatusSignal<Current> acceleratorCurrentAmps;
  private final StatusSignal<Temperature> acceleratorTemperatureCelsius;

  private final TalonFXConfiguration motorConfig;

  private final NeutralOut neutralControl;
  private final VoltageOut voltageControl;

  public WhiplashIntakeIOTalonFX() {
    topMotor = new TalonFX(WhiplashIntakeConstants.TOP_CAN_ID);
    bottomMotor = new TalonFX(WhiplashIntakeConstants.BOTTOM_CAN_ID);
    acceleratorMotor = new TalonFX(WhiplashIntakeConstants.ACCELERATOR_CAN_ID);

    intakeSensor = new DigitalInput(WhiplashIntakeConstants.INTAKE_SENSOR_CHANNEL);
    middleSensor = new DigitalInput(WhiplashIntakeConstants.MIDDLE_SENSOR_CHANNEL);
    finalSensor = new DigitalInput(WhiplashIntakeConstants.FINAL_SENSOR_CHANNEL);

    motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.StatorCurrentLimit = WhiplashIntakeConstants.CURRENT_LIMIT;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    topMotor.getConfigurator().apply(motorConfig);
    bottomMotor.getConfigurator().apply(motorConfig);
    acceleratorMotor.getConfigurator().apply(motorConfig);

    topPositionRotations = topMotor.getPosition();
    topVelocityRotPerSec = topMotor.getVelocity();
    topAppliedVolts = topMotor.getMotorVoltage();
    topCurrentAmps = topMotor.getSupplyCurrent();
    topTemperatureCelsius = topMotor.getDeviceTemp();

    bottomPositionRotations = bottomMotor.getPosition();
    bottomVelocityRotPerSec = bottomMotor.getVelocity();
    bottomAppliedVolts = bottomMotor.getMotorVoltage();
    bottomCurrentAmps = bottomMotor.getSupplyCurrent();
    bottomTemperatureCelsius = bottomMotor.getDeviceTemp();

    acceleratorPositionRotations = acceleratorMotor.getPosition();
    acceleratorVelocityRotPerSec = acceleratorMotor.getVelocity();
    acceleratorAppliedVolts = acceleratorMotor.getMotorVoltage();
    acceleratorCurrentAmps = acceleratorMotor.getSupplyCurrent();
    acceleratorTemperatureCelsius = acceleratorMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        topPositionRotations,
        topVelocityRotPerSec,
        topAppliedVolts,
        topCurrentAmps,
        topTemperatureCelsius,
        bottomPositionRotations,
        bottomVelocityRotPerSec,
        bottomAppliedVolts,
        bottomCurrentAmps,
        bottomTemperatureCelsius,
        acceleratorPositionRotations,
        acceleratorVelocityRotPerSec,
        acceleratorAppliedVolts,
        acceleratorCurrentAmps,
        acceleratorTemperatureCelsius);
    topMotor.optimizeBusUtilization();
    bottomMotor.optimizeBusUtilization();
    acceleratorMotor.optimizeBusUtilization();

    neutralControl = new NeutralOut();
    voltageControl = new VoltageOut(0.0);
  }

  @Override
  public void updateInputs(WhiplashIntakeIOInputs inputs) {
    inputs.topPosition = Rotation2d.fromRotations(topPositionRotations.getValueAsDouble());
    inputs.topVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(topVelocityRotPerSec.getValueAsDouble());
    inputs.topAppliedVolts = topAppliedVolts.getValueAsDouble();
    inputs.topCurrentAmps = topCurrentAmps.getValueAsDouble();
    inputs.topTemperatureCelsius = topTemperatureCelsius.getValueAsDouble();

    inputs.bottomPosition = Rotation2d.fromRotations(bottomPositionRotations.getValueAsDouble());
    inputs.bottomVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(bottomCurrentAmps.getValueAsDouble());
    inputs.bottomAppliedVolts = bottomAppliedVolts.getValueAsDouble();
    inputs.bottomCurrentAmps = bottomCurrentAmps.getValueAsDouble();
    inputs.bottomTemperatureCelsius = bottomTemperatureCelsius.getValueAsDouble();

    inputs.acceleratorPosition =
        Rotation2d.fromRotations(acceleratorPositionRotations.getValueAsDouble());
    inputs.acceleratorVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(acceleratorCurrentAmps.getValueAsDouble());
    inputs.acceleratorAppliedVolts = acceleratorAppliedVolts.getValueAsDouble();
    inputs.acceleratorCurrentAmps = acceleratorCurrentAmps.getValueAsDouble();
    inputs.acceleratorTemperatureCelsius = acceleratorTemperatureCelsius.getValueAsDouble();

    inputs.intakeSensor = !intakeSensor.get();
    inputs.middleSensor = !middleSensor.get();
    inputs.finalSensor = !finalSensor.get();
  }

  @Override
  public void setTopVoltage(double volts) {
    topMotor.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void setBottomVoltage(double volts) {
    bottomMotor.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void setAcceleratorVoltage(double volts) {
    acceleratorMotor.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void stop() {
    topMotor.setControl(neutralControl);
    bottomMotor.setControl(neutralControl);
    acceleratorMotor.setControl(neutralControl);
  }
}
