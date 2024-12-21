package frc.robot.subsystems.snapback.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class SnapbackIntakeIOTalonFX implements SnapbackIntakeIO {
  private final TalonFX intakeMotor;
  private final StatusSignal<Angle> intakePosition;
  private final StatusSignal<AngularVelocity> intakeVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> intakeAppliedVolts;
  private final StatusSignal<Current> intakeCurrentAmps;
  private final StatusSignal<Temperature> intakeTemperatureCelsius;

  private final TalonFX serializerMotor;
  private final StatusSignal<Angle> serializerPosition;
  private final StatusSignal<AngularVelocity> serializerVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> serializerAppliedVolts;
  private final StatusSignal<Current> serializerCurrentAmps;
  private final StatusSignal<Temperature> serializerTemperatureCelsius;

  private final TalonFX kickerMotor;
  private final StatusSignal<Angle> kickerPosition;
  private final StatusSignal<AngularVelocity> kickerVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> kickerAppliedVolts;
  private final StatusSignal<Current> kickerCurrentAmps;
  private final StatusSignal<Temperature> kickerTemperatureCelsius;

  private final DigitalInput sensor;

  private final DoubleSolenoid pneumatic;

  private final VoltageOut voltageControlRequest;

  public SnapbackIntakeIOTalonFX() {
    intakeMotor = new TalonFX(SnapbackIntakeConstants.INTAKE_MOTOR_CAN_ID);
    serializerMotor = new TalonFX(SnapbackIntakeConstants.SERIALIZER_MOTOR_CAN_ID);
    kickerMotor = new TalonFX(SnapbackIntakeConstants.KICKER_MOTOR_CAN_ID);
    sensor = new DigitalInput(SnapbackIntakeConstants.SENSOR_CHANNEL);
    pneumatic =
        new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            SnapbackIntakeConstants.PNEUMATIC_FORWARD_CHANNEL,
            SnapbackIntakeConstants.PNEUMATIC_REVERSE_CHANNEL);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    TalonFXConfiguration intakeConfig = config;
    intakeConfig.Feedback.SensorToMechanismRatio = SnapbackIntakeConstants.INTAKE_GEAR_REDUCTION;
    intakeMotor.getConfigurator().apply(intakeConfig);

    TalonFXConfiguration serializerConfig = config;
    serializerConfig.Feedback.SensorToMechanismRatio =
        SnapbackIntakeConstants.SERIALIZER_GEAR_REDUCTION;
    serializerMotor.getConfigurator().apply(serializerConfig);

    TalonFXConfiguration kickerConfig = config;
    kickerConfig.Feedback.SensorToMechanismRatio = SnapbackIntakeConstants.KICKER_GEAR_REDUCTION;
    kickerMotor.getConfigurator().apply(kickerConfig);

    intakePosition = intakeMotor.getPosition();
    intakeVelocityRotationsPerSecond = intakeMotor.getVelocity();
    intakeAppliedVolts = intakeMotor.getMotorVoltage();
    intakeCurrentAmps = intakeMotor.getSupplyCurrent();
    intakeTemperatureCelsius = intakeMotor.getDeviceTemp();

    serializerPosition = serializerMotor.getPosition();
    serializerVelocityRotationsPerSecond = serializerMotor.getVelocity();
    serializerAppliedVolts = serializerMotor.getMotorVoltage();
    serializerCurrentAmps = serializerMotor.getSupplyCurrent();
    serializerTemperatureCelsius = serializerMotor.getDeviceTemp();

    kickerPosition = kickerMotor.getPosition();
    kickerVelocityRotationsPerSecond = kickerMotor.getVelocity();
    kickerAppliedVolts = kickerMotor.getMotorVoltage();
    kickerCurrentAmps = kickerMotor.getSupplyCurrent();
    kickerTemperatureCelsius = kickerMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        intakeAppliedVolts,
        intakeCurrentAmps,
        intakeTemperatureCelsius,
        intakeVelocityRotationsPerSecond,
        serializerAppliedVolts,
        serializerCurrentAmps,
        serializerTemperatureCelsius,
        serializerVelocityRotationsPerSecond,
        kickerAppliedVolts,
        kickerCurrentAmps,
        kickerTemperatureCelsius,
        kickerVelocityRotationsPerSecond);

    intakeMotor.optimizeBusUtilization();
    serializerMotor.optimizeBusUtilization();
    kickerMotor.optimizeBusUtilization();

    voltageControlRequest = new VoltageOut(0.0);
  }

  @Override
  public void updateInputs(SnapbackIntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        intakePosition,
        intakeAppliedVolts,
        intakeCurrentAmps,
        intakeTemperatureCelsius,
        intakeVelocityRotationsPerSecond,
        serializerPosition,
        serializerAppliedVolts,
        serializerCurrentAmps,
        serializerTemperatureCelsius,
        serializerVelocityRotationsPerSecond,
        kickerPosition,
        kickerAppliedVolts,
        kickerCurrentAmps,
        kickerTemperatureCelsius,
        kickerVelocityRotationsPerSecond);

    inputs.intakePosition = Rotation2d.fromRotations(intakePosition.getValueAsDouble());
    inputs.intakeVelocityRadiansPerSecond =
        Units.rotationsToRadians(intakeVelocityRotationsPerSecond.getValueAsDouble());
    inputs.intakeAppliedVolts = intakeAppliedVolts.getValueAsDouble();
    inputs.intakeCurrentAmps = intakeCurrentAmps.getValueAsDouble();
    inputs.intakeTemperatureCelsius = intakeTemperatureCelsius.getValueAsDouble();

    inputs.serializerPosition = Rotation2d.fromRotations(serializerPosition.getValueAsDouble());
    inputs.serializerVelocityRadiansPerSecond =
        serializerVelocityRotationsPerSecond.getValueAsDouble();
    inputs.serializerAppliedVolts = serializerAppliedVolts.getValueAsDouble();
    inputs.serializerCurrentAmps = serializerCurrentAmps.getValueAsDouble();
    inputs.serializerTemperatureCelsius = serializerTemperatureCelsius.getValueAsDouble();

    inputs.kickerPosition = Rotation2d.fromRotations(kickerPosition.getValueAsDouble());
    inputs.kickerVelocityRadiansPerSecond = kickerVelocityRotationsPerSecond.getValueAsDouble();
    inputs.kickerAppliedVolts = kickerAppliedVolts.getValueAsDouble();
    inputs.kickerCurrentAmps = kickerCurrentAmps.getValueAsDouble();
    inputs.kickerTemperatureCelsius = kickerTemperatureCelsius.getValueAsDouble();

    inputs.sensorValue = sensor.get();
    inputs.pneumaticValue = pneumatic.get();
  }

  @Override
  public void setIntakeVoltage(double volts) {
    intakeMotor.setControl(voltageControlRequest.withOutput(volts).withUpdateFreqHz(1000.0));
  }

  @Override
  public void setSerializerVoltage(double volts) {
    serializerMotor.setControl(voltageControlRequest.withOutput(volts).withUpdateFreqHz(1000.0));
  }

  @Override
  public void setKickerVoltage(double volts) {
    kickerMotor.setControl(voltageControlRequest.withOutput(volts).withUpdateFreqHz(1000.0));
  }

  @Override
  public void setActuatorValue(DoubleSolenoid.Value value) {
    pneumatic.set(value);
  }
}
