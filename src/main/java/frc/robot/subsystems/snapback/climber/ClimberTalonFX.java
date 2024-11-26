package frc.robot.subsystems.snapback.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
  public StatusSignal<Double> leftPositionGoalMeters;
  public StatusSignal<Double> leftPositionErrorMeters;

  public StatusSignal<Angle> rightPositionRotations;
  public StatusSignal<AngularVelocity> rightVelocityRotationsPerSecond;
  public StatusSignal<Voltage> rightAppliedVolts;
  public StatusSignal<Current> rightCurrentAmps;
  public StatusSignal<Temperature> rightTemperatureCelsius;
  public StatusSignal<Double> rightPositionGoalMeters;
  public StatusSignal<Double> rightPositionErrorMeters;

  private final Alert leftDisconnectedAlert =
      new Alert("Left climber TalonFX disconnected, check CAN bus.", AlertType.ERROR);
  private final Alert rightDisconnectedAlert =
      new Alert("Right climber TalonFX disconnected, check CAN bus.", AlertType.ERROR);

  private VoltageOut voltageControlRequest;
  private MotionMagicVoltage positionControlRequest;

  public ClimberTalonFX() {
    leftClimber = new TalonFX(ClimberConstants.LEFT_CLIMBER_MOTOR_CAN_ID);
    rightClimber = new TalonFX(ClimberConstants.RIGHT_CLIMBER_MOTOR_CAN_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = ClimberConstants.CLIMBER_GEAR_REDUCTION;
    config.Slot0.kP = ClimberConstants.GAINS.kp().get();
    config.Slot0.kI = ClimberConstants.GAINS.ki().get();
    config.Slot0.kD = ClimberConstants.GAINS.kd().get();
    config.Slot0.kS = ClimberConstants.GAINS.ks();
    config.Slot0.kV = ClimberConstants.GAINS.kv();
    config.Slot0.kA = ClimberConstants.GAINS.ka();
    config.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.CRUISE_VELOCITY.get();
    config.MotionMagic.MotionMagicAcceleration = ClimberConstants.MAX_ACCELERATION.get();
    leftClimber.getConfigurator().apply(config);
    rightClimber.getConfigurator().apply(config);

    leftPositionRotations = leftClimber.getPosition();
    leftVelocityRotationsPerSecond = leftClimber.getVelocity();
    leftAppliedVolts = leftClimber.getMotorVoltage();
    leftCurrentAmps = leftClimber.getSupplyCurrent();
    leftTemperatureCelsius = leftClimber.getDeviceTemp();
    leftPositionGoalMeters = leftClimber.getClosedLoopReference();
    leftPositionErrorMeters = leftClimber.getClosedLoopError();

    rightPositionRotations = rightClimber.getPosition();
    rightVelocityRotationsPerSecond = rightClimber.getVelocity();
    rightAppliedVolts = rightClimber.getMotorVoltage();
    rightCurrentAmps = rightClimber.getSupplyCurrent();
    rightTemperatureCelsius = rightClimber.getDeviceTemp();
    rightPositionGoalMeters = rightClimber.getClosedLoopReference();
    rightPositionErrorMeters = rightClimber.getClosedLoopError();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        leftPositionRotations,
        leftVelocityRotationsPerSecond,
        leftAppliedVolts,
        leftCurrentAmps,
        leftTemperatureCelsius,
        leftPositionGoalMeters,
        leftPositionErrorMeters,
        rightPositionRotations,
        rightVelocityRotationsPerSecond,
        rightAppliedVolts,
        rightCurrentAmps,
        rightTemperatureCelsius,
        rightPositionGoalMeters,
        rightPositionErrorMeters);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    boolean leftConnected =
        BaseStatusSignal.refreshAll(
                leftPositionRotations,
                leftVelocityRotationsPerSecond,
                leftAppliedVolts,
                leftCurrentAmps,
                leftTemperatureCelsius,
                leftPositionGoalMeters,
                leftPositionErrorMeters)
            .isOK();
    boolean rightConnected =
        BaseStatusSignal.refreshAll(
                rightPositionRotations,
                rightVelocityRotationsPerSecond,
                rightAppliedVolts,
                rightCurrentAmps,
                rightTemperatureCelsius,
                rightPositionGoalMeters,
                rightPositionErrorMeters)
            .isOK();

    leftDisconnectedAlert.set(!leftConnected);
    rightDisconnectedAlert.set(!rightConnected);

    inputs.leftPosition =
        leftPositionRotations.getValueAsDouble()
            * 2
            * Math.PI
            * ClimberConstants.CLIMBER_PULLY_RADIUS_METERS;
    inputs.leftVelocityRadiansPerSecond =
        Units.rotationsToRadians(leftVelocityRotationsPerSecond.getValueAsDouble());
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftCurrentAmps = leftCurrentAmps.getValueAsDouble();
    inputs.leftTemperatureCelsius = leftTemperatureCelsius.getValueAsDouble();
    inputs.leftPositionGoalMeters = leftPositionGoalMeters.getValueAsDouble();
    inputs.leftPositionErrorMeters = leftPositionErrorMeters.getValueAsDouble();

    inputs.rightPosition =
        rightPositionRotations.getValueAsDouble()
            * 2
            * Math.PI
            * ClimberConstants.CLIMBER_PULLY_RADIUS_METERS;
    inputs.rightVelocityRadiansPerSecond =
        Units.rotationsToRadians(rightVelocityRotationsPerSecond.getValueAsDouble());
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightCurrentAmps = rightCurrentAmps.getValueAsDouble();
    inputs.rightTemperatureCelsius = rightTemperatureCelsius.getValueAsDouble();
    inputs.rightPositionGoalMeters = rightPositionGoalMeters.getValueAsDouble();
    inputs.rightPositionErrorMeters = rightPositionErrorMeters.getValueAsDouble();
  }

  @Override
  public void setLeftVoltage(double volts) {
    leftClimber.setControl(voltageControlRequest.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setRightVoltage(double volts) {
    rightClimber.setControl(voltageControlRequest.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setLeftPositionGoal(double positionMeters) {
    double positionRotations =
        positionMeters / (2 * Math.PI * ClimberConstants.CLIMBER_PULLY_RADIUS_METERS);
    leftClimber.setControl(
        positionControlRequest.withPosition(positionRotations).withEnableFOC(true));
  }

  @Override
  public void setRightPositionGoal(double positionMeters) {
    double positionRotations =
        positionMeters / (2 * Math.PI * ClimberConstants.CLIMBER_PULLY_RADIUS_METERS);
    rightClimber.setControl(
        positionControlRequest.withPosition(positionRotations).withEnableFOC(true));
  }

  @Override
  public void setLeftPosition(double positionMeters) {
    double positionRotations =
        positionMeters / (2 * Math.PI * ClimberConstants.CLIMBER_PULLY_RADIUS_METERS);
    leftClimber.setPosition(positionRotations);
  }

  @Override
  public void setRightPosition(double positionMeters) {
    double positionRotations =
        positionMeters / (2 * Math.PI * ClimberConstants.CLIMBER_PULLY_RADIUS_METERS);
    rightClimber.setPosition(positionRotations);
  }

  @Override
  public boolean atGoal() {
    double leftPositionMeters =
        leftPositionRotations.getValueAsDouble()
            * 2
            * Math.PI
            * ClimberConstants.CLIMBER_PULLY_RADIUS_METERS;
    double rightPositionMeters =
        rightPositionRotations.getValueAsDouble()
            * 2
            * Math.PI
            * ClimberConstants.CLIMBER_PULLY_RADIUS_METERS;

    return Math.abs(
                leftPositionMeters
                    - leftPositionGoalMeters.getValueAsDouble()
                        * 2
                        * Math.PI
                        * ClimberConstants.CLIMBER_PULLY_RADIUS_METERS)
            < ClimberConstants.CLIMBER_TOLERANCE_METERS.get()
        && Math.abs(
                rightPositionMeters
                    - rightPositionGoalMeters.getValueAsDouble()
                        * 2
                        * Math.PI
                        * ClimberConstants.CLIMBER_PULLY_RADIUS_METERS)
            < ClimberConstants.CLIMBER_TOLERANCE_METERS.get();
  }
}
