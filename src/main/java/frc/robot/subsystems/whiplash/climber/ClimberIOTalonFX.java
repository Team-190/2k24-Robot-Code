package frc.robot.subsystems.whiplash.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX climber;

  public StatusSignal<Angle> climberPositionRotations;
  public StatusSignal<AngularVelocity> climberVelocityRotationsPerSecond;
  public StatusSignal<Voltage> climberAppliedVolts;
  public StatusSignal<Current> climberCurrentAmps;
  public StatusSignal<Temperature> climberTemperatureCelsius;

  private final Alert climberDisconnectedAlert =
      new Alert("Climber TalonFX disconnected, check CAN bus.", AlertType.ERROR);

  private VoltageOut voltageControlRequest;

  public ClimberIOTalonFX() {
     climber = new TalonFX(ClimberConstants.CLIMBER_MOTOR_CAN_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = ClimberConstants.CLIMBER_GEAR_REDUCTION;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ClimberConstants.CLIMBER_MAX_HEIGHT_METERS
            * 2.0
            * Math.PI
            / ClimberConstants.CLIMBER_PULLEY_RADIUS_METERS;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        ClimberConstants.CLIMBER_MIN_HEIGHT_METERS
            * 2.0
            * Math.PI
            / ClimberConstants.CLIMBER_PULLEY_RADIUS_METERS;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    climber.getConfigurator().apply(config);

    climberPositionRotations = climber.getPosition();
    climberVelocityRotationsPerSecond = climber.getVelocity();
    climberAppliedVolts = climber.getMotorVoltage();
    climberCurrentAmps = climber.getSupplyCurrent();
    climberTemperatureCelsius = climber.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        climberPositionRotations,
        climberVelocityRotationsPerSecond,
        climberAppliedVolts,
        climberCurrentAmps,
        climberTemperatureCelsius);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    boolean climberConnected =
        BaseStatusSignal.refreshAll(
                climberPositionRotations,
                climberVelocityRotationsPerSecond,
                climberAppliedVolts,
                climberCurrentAmps,
                climberTemperatureCelsius)
            .isOK();

    climberDisconnectedAlert.set(!climberConnected);

    inputs.climberPositionMeters =
        Units.rotationsToRadians(climberPositionRotations.getValueAsDouble())
            * 2
            * Math.PI
            * ClimberConstants.CLIMBER_PULLEY_RADIUS_METERS;
    inputs.climberVelocityMetersPerSecond =
        Units.rotationsToRadians(climberVelocityRotationsPerSecond.getValueAsDouble());
    inputs.climberAppliedVolts = climberAppliedVolts.getValueAsDouble();
    inputs.climberCurrentAmps = climberCurrentAmps.getValueAsDouble();
    inputs.climberTemperatureCelsius = climberTemperatureCelsius.getValueAsDouble();
  }

  @Override
  public void setClimberVoltage(double volts) {
    climber.setControl(voltageControlRequest.withOutput(volts).withUpdateFreqHz(1000.0));
  }
}
