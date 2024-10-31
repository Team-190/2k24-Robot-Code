package frc.robot.subsystems.snapback.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX intakeMotor;
  public StatusSignal<Double> intakePosition;
  public StatusSignal<Double> intakeVelocityRadiansPerSecond;
  public StatusSignal<Double> intakeAppliedVolts;
  public StatusSignal<Double> intakeCurrentAmps;
  public StatusSignal<Double> intakeTemperatureCelsius;

  private final TalonFX serializerMotor;
  public StatusSignal<Double> serializerPosition;
  public StatusSignal<Double> serializerVelocityRadiansPerSecond;
  public StatusSignal<Double> serializerAppliedVolts;
  public StatusSignal<Double> serializerCurrentAmps;
  public StatusSignal<Double> serializerTemperatureCelsius;

  private final TalonFX kickerMotor;
  public StatusSignal<Double> kickerPosition;
  public StatusSignal<Double> kickerVelocityRadiansPerSecond;
  public StatusSignal<Double> kickerAppliedVolts;
  public StatusSignal<Double> kickerCurrentAmps;
  public StatusSignal<Double> kickerTemperatureCelsius;

  private final DigitalInput sensor;

  private final DoubleSolenoid pneumatic;

  private VoltageOut voltageControl;

  public IntakeIOTalonFX() {
    intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
    serializerMotor = new TalonFX(IntakeConstants.SERIALIZER_MOTOR_ID);
    kickerMotor = new TalonFX(IntakeConstants.KICKER_MOTOR_ID);
    sensor = new DigitalInput(IntakeConstants.SENSOR_ID);
    pneumatic =
        new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            IntakeConstants.PNEUMATIC_ID,
            IntakeConstants.PNEUMATIC_ID + 1);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    TalonFXConfiguration intakeConfig = config;
    intakeConfig.Feedback.SensorToMechanismRatio = IntakeConstants.INTAKE_GEAR_RATIO;
    intakeMotor.getConfigurator().apply(intakeConfig);

    TalonFXConfiguration serializerConfig = config;
    serializerConfig.Feedback.SensorToMechanismRatio = IntakeConstants.SERIALIZER_GEAR_RATIO;
    serializerMotor.getConfigurator().apply(serializerConfig);

    TalonFXConfiguration kickerConfig = config;
    kickerConfig.Feedback.SensorToMechanismRatio = IntakeConstants.KICKER_GEAR_RATIO;
    kickerMotor.getConfigurator().apply(kickerConfig);

    intakePosition = intakeMotor.getPosition();
    intakeVelocityRadiansPerSecond = intakeMotor.getVelocity();
    intakeAppliedVolts = intakeMotor.getMotorVoltage();
    intakeCurrentAmps = intakeMotor.getSupplyCurrent();
    intakeTemperatureCelsius = intakeMotor.getDeviceTemp();

    serializerPosition = serializerMotor.getPosition();
    serializerVelocityRadiansPerSecond = serializerMotor.getVelocity();
    serializerAppliedVolts = serializerMotor.getMotorVoltage();
    serializerCurrentAmps = serializerMotor.getSupplyCurrent();
    serializerTemperatureCelsius = serializerMotor.getDeviceTemp();

    kickerPosition = kickerMotor.getPosition();
    kickerVelocityRadiansPerSecond = kickerMotor.getVelocity();
    kickerAppliedVolts = kickerMotor.getMotorVoltage();
    kickerCurrentAmps = kickerMotor.getSupplyCurrent();
    kickerTemperatureCelsius = kickerMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        intakeAppliedVolts,
        intakeCurrentAmps,
        intakeTemperatureCelsius,
        intakeVelocityRadiansPerSecond,
        serializerAppliedVolts,
        serializerCurrentAmps,
        serializerTemperatureCelsius,
        serializerVelocityRadiansPerSecond,
        kickerAppliedVolts,
        kickerCurrentAmps,
        kickerTemperatureCelsius,
        kickerVelocityRadiansPerSecond);

    voltageControl = new VoltageOut(0.0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        intakePosition,
        intakeAppliedVolts,
        intakeCurrentAmps,
        intakeTemperatureCelsius,
        intakeVelocityRadiansPerSecond,
        serializerPosition,
        serializerAppliedVolts,
        serializerCurrentAmps,
        serializerTemperatureCelsius,
        serializerVelocityRadiansPerSecond,
        kickerPosition,
        kickerAppliedVolts,
        kickerCurrentAmps,
        kickerTemperatureCelsius,
        kickerVelocityRadiansPerSecond);

    inputs.intakePosition = Rotation2d.fromRotations(intakePosition.getValueAsDouble());
    inputs.intakeVelocityRadiansPerSecond =
        Units.rotationsToRadians(intakeVelocityRadiansPerSecond.getValueAsDouble());
    inputs.intakeAppliedVolts = intakeAppliedVolts.getValueAsDouble();
    inputs.intakeCurrentAmps = intakeCurrentAmps.getValueAsDouble();
    inputs.intakeTemperatureCelsius = intakeTemperatureCelsius.getValueAsDouble();

    inputs.serializerPosition = Rotation2d.fromRotations(serializerPosition.getValueAsDouble());
    inputs.serializerVelocityRadiansPerSecond =
        serializerVelocityRadiansPerSecond.getValueAsDouble();
    inputs.serializerAppliedVolts = serializerAppliedVolts.getValueAsDouble();
    inputs.serializerCurrentAmps = serializerCurrentAmps.getValueAsDouble();
    inputs.serializerTemperatureCelsius = serializerTemperatureCelsius.getValueAsDouble();

    inputs.kickerPosition = Rotation2d.fromRotations(kickerPosition.getValueAsDouble());
    inputs.kickerVelocityRadiansPerSecond = kickerVelocityRadiansPerSecond.getValueAsDouble();
    inputs.kickerAppliedVolts = kickerAppliedVolts.getValueAsDouble();
    inputs.kickerCurrentAmps = kickerCurrentAmps.getValueAsDouble();
    inputs.kickerTemperatureCelsius = kickerTemperatureCelsius.getValueAsDouble();

    inputs.sensorValue = sensor.get();
    inputs.pneumaticValue = pneumatic.get();
  }

  @Override
  public void setIntakeVoltage(double volts) {
    intakeMotor.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void setSerializerVoltage(double volts) {
    serializerMotor.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void setKickerVoltage(double volts) {
    kickerMotor.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void setActuatorValue(DoubleSolenoid.Value value) {
    pneumatic.set(value);
  }
}
