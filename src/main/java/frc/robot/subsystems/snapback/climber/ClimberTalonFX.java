package frc.robot.subsystems.snapback.climber;

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

public class ClimberTalonFX implements ClimberIO {
  private final TalonFX leftClimber;
  private final TalonFX rightClimber;

  public StatusSignal<Angle> leftPositionRotations;
  public StatusSignal<AngularVelocity> leftVelocityRotationsPerSecond;
  public StatusSignal<Voltage> leftAppliedVolts;
  public StatusSignal<Current> leftCurrentAmps;
  public StatusSignal<Temperature> leftTemperatureCelsius;

  public StatusSignal<Angle> rightPositionRotations;
  public StatusSignal<AngularVelocity> rightVelocityRotationsPerSecond;
  public StatusSignal<Voltage> rightAppliedVolts;
  public StatusSignal<Current> rightCurrentAmps;
  public StatusSignal<Temperature> rightTemperatureCelsius;

  private final Alert leftDisconnectedAlert =
      new Alert("Left climber TalonFX disconnected, check CAN bus.", AlertType.ERROR);
  private final Alert rightDisconnectedAlert =
      new Alert("Right climber TalonFX disconnected, check CAN bus.", AlertType.ERROR);

  private VoltageOut voltageControlRequest;

  public ClimberTalonFX() {
    leftClimber = new TalonFX(ClimberConstants.LEFT_CLIMBER_MOTOR_CAN_ID);
    rightClimber = new TalonFX(ClimberConstants.RIGHT_CLIMBER_MOTOR_CAN_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = ClimberConstants.CLIMBER_GEAR_REDUCTION;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ClimberConstants.CLIMBER_MAX_HEIGHT_METERS
            * 2.0
            * Math.PI
            / ClimberConstants.CLIMBER_PULLY_RADIUS_METERS;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        ClimberConstants.CLIMBER_MIN_HEIGHT_METERS
            * 2.0
            * Math.PI
            / ClimberConstants.CLIMBER_PULLY_RADIUS_METERS;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    leftClimber.getConfigurator().apply(config);
    rightClimber.getConfigurator().apply(config);

    leftPositionRotations = leftClimber.getPosition();
    leftVelocityRotationsPerSecond = leftClimber.getVelocity();
    leftAppliedVolts = leftClimber.getMotorVoltage();
    leftCurrentAmps = leftClimber.getSupplyCurrent();
    leftTemperatureCelsius = leftClimber.getDeviceTemp();

    rightPositionRotations = rightClimber.getPosition();
    rightVelocityRotationsPerSecond = rightClimber.getVelocity();
    rightAppliedVolts = rightClimber.getMotorVoltage();
    rightCurrentAmps = rightClimber.getSupplyCurrent();
    rightTemperatureCelsius = rightClimber.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        leftPositionRotations,
        leftVelocityRotationsPerSecond,
        leftAppliedVolts,
        leftCurrentAmps,
        leftTemperatureCelsius,
        rightPositionRotations,
        rightVelocityRotationsPerSecond,
        rightAppliedVolts,
        rightCurrentAmps,
        rightTemperatureCelsius);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    boolean leftConnected =
        BaseStatusSignal.refreshAll(
                leftPositionRotations,
                leftVelocityRotationsPerSecond,
                leftAppliedVolts,
                leftCurrentAmps,
                leftTemperatureCelsius)
            .isOK();
    boolean rightConnected =
        BaseStatusSignal.refreshAll(
                rightPositionRotations,
                rightVelocityRotationsPerSecond,
                rightAppliedVolts,
                rightCurrentAmps,
                rightTemperatureCelsius)
            .isOK();

    leftDisconnectedAlert.set(!leftConnected);
    rightDisconnectedAlert.set(!rightConnected);

    inputs.leftPositionMeters =
        Units.rotationsToRadians(leftPositionRotations.getValueAsDouble())
            * 2
            * Math.PI
            * ClimberConstants.CLIMBER_PULLY_RADIUS_METERS;
    inputs.leftVelocityMetersPerSecond =
        Units.rotationsToRadians(leftVelocityRotationsPerSecond.getValueAsDouble());
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftCurrentAmps = leftCurrentAmps.getValueAsDouble();
    inputs.leftTemperatureCelsius = leftTemperatureCelsius.getValueAsDouble();

    inputs.rightPositionMeters =
        Units.rotationsToRadians(rightPositionRotations.getValueAsDouble())
            * 2
            * Math.PI
            * ClimberConstants.CLIMBER_PULLY_RADIUS_METERS;
    inputs.rightVelocityMetersPerSecond =
        Units.rotationsToRadians(rightVelocityRotationsPerSecond.getValueAsDouble());
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightCurrentAmps = rightCurrentAmps.getValueAsDouble();
    inputs.rightTemperatureCelsius = rightTemperatureCelsius.getValueAsDouble();
  }

  @Override
  public void setLeftVoltage(double volts) {
    leftClimber.setControl(voltageControlRequest.withOutput(volts).withUpdateFreqHz(1000.0));
  }

  @Override
  public void setRightVoltage(double volts) {
    rightClimber.setControl(voltageControlRequest.withOutput(volts).withUpdateFreqHz(1000.0));
  }
}
