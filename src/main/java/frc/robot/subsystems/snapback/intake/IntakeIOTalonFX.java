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

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX intakeMotor;
  public StatusSignal<Angle> intakePosition;
  public StatusSignal<AngularVelocity> intakeVelocityRotationsPerSecond;
  public StatusSignal<Voltage> intakeAppliedVolts;
  public StatusSignal<Current> intakeCurrentAmps;
  public StatusSignal<Temperature> intakeTemperatureCelsius;

  private final TalonFX serializerMotor;
  public StatusSignal<Angle> serializerPosition;
  public StatusSignal<AngularVelocity> serializerVelocityRotationsPerSecond;
  public StatusSignal<Voltage> serializerAppliedVolts;
  public StatusSignal<Current> serializerCurrentAmps;
  public StatusSignal<Temperature> serializerTemperatureCelsius;

  private final TalonFX kickerMotor;
  public StatusSignal<Angle> kickerPosition;
  public StatusSignal<AngularVelocity> kickerVelocityRotationsPerSecond;
  public StatusSignal<Voltage> kickerAppliedVolts;
  public StatusSignal<Current> kickerCurrentAmps;
  public StatusSignal<Temperature> kickerTemperatureCelsius;

  private final DigitalInput sensor;

  private final DoubleSolenoid pneumatic;

  private VoltageOut voltageControl;

  public IntakeIOTalonFX() {
    intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_CAN_ID);
    serializerMotor = new TalonFX(IntakeConstants.SERIALIZER_MOTOR_CAN_ID);
    kickerMotor = new TalonFX(IntakeConstants.KICKER_MOTOR_CAN_ID);
    sensor = new DigitalInput(IntakeConstants.SENSOR_CHANNEL);
    pneumatic =
        new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            IntakeConstants.PNEUMATIC_FORWARD_CHANNEL,
            IntakeConstants.PNEUMATIC_REVERSE_CHANNEL);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    TalonFXConfiguration intakeConfig = config;
    intakeConfig.Feedback.SensorToMechanismRatio = IntakeConstants.INTAKE_GEAR_REDUCTION;
    intakeMotor.getConfigurator().apply(intakeConfig);

    TalonFXConfiguration serializerConfig = config;
    serializerConfig.Feedback.SensorToMechanismRatio = IntakeConstants.SERIALIZER_GEAR_REDUCTION;
    serializerMotor.getConfigurator().apply(serializerConfig);

    TalonFXConfiguration kickerConfig = config;
    kickerConfig.Feedback.SensorToMechanismRatio = IntakeConstants.KICKER_GEAR_REDUCTION;
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

    intakeMotor.optimizeBusUtilization(50.0, 1.0);
    serializerMotor.optimizeBusUtilization(50.0, 1.0);
    kickerMotor.optimizeBusUtilization(50.0, 1.0);

    voltageControl = new VoltageOut(0.0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
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
    intakeMotor.setControl(voltageControl.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setSerializerVoltage(double volts) {
    serializerMotor.setControl(voltageControl.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setKickerVoltage(double volts) {
    kickerMotor.setControl(voltageControl.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setActuatorValue(DoubleSolenoid.Value value) {
    pneumatic.set(value);
  }
}
