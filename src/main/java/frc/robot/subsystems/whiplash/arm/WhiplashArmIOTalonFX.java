package frc.robot.subsystems.whiplash.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class WhiplashArmIOTalonFX implements WhiplashArmIO {
  private final TalonFX motor;
  private final CANcoder cancoder;

  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocityRotationsPerSecond;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;
  private final StatusSignal<Temperature> temperatureCelcius;
  private Rotation2d positionGoal;
  private final StatusSignal<Double> positionSetpointRotations;
  private final StatusSignal<Double> positionErrorRotations;

  private final StatusSignal<Angle> absolutePosition;

  private final NeutralOut neutralControl;
  private final VoltageOut voltageControl;
  private final MotionMagicVoltage voltagePositionControl;

  private final TalonFXConfiguration motorConfig;
  private final CANcoderConfiguration cancoderConfig;

  public WhiplashArmIOTalonFX() {
    motor = new TalonFX(WhiplashArmConstants.MOTOR_CAN_ID);
    cancoder = new CANcoder(WhiplashArmConstants.CANCODER_CAN_ID);

    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.CurrentLimits.StatorCurrentLimit = 60.0;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    motorConfig.Feedback.SensorToMechanismRatio = 1.0;
    motorConfig.Feedback.RotorToSensorRatio = WhiplashArmConstants.GEAR_RATIO;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity =
        Units.radiansToRotations(
            WhiplashArmConstants.CONSTRAINTS.maxVelocityRadiansPerSecond().get());
    motorConfig.MotionMagic.MotionMagicAcceleration =
        Units.radiansToRotations(
            WhiplashArmConstants.CONSTRAINTS.maxAccelerationRadiansPerSecondSqaured().get());
    motorConfig.Slot0.kP = WhiplashArmConstants.GAINS.kp().get();
    motorConfig.Slot0.kD = WhiplashArmConstants.GAINS.kd().get();
    motorConfig.Slot0.kS = WhiplashArmConstants.GAINS.ks().get();
    motorConfig.Slot0.kG = WhiplashArmConstants.GAINS.kg().get();
    motorConfig.Slot0.kV = WhiplashArmConstants.GAINS.kv().get();
    motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    motor.getConfigurator().apply(motorConfig);

    cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    cancoderConfig.MagnetSensor.MagnetOffset =
        WhiplashArmConstants.ABSOLUTE_ENCODER_OFFSET.getRotations();
    cancoder.getConfigurator().apply(cancoderConfig);

    positionRotations = motor.getPosition();
    velocityRotationsPerSecond = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    currentAmps = motor.getSupplyCurrent();
    temperatureCelcius = motor.getDeviceTemp();
    positionGoal = new Rotation2d();
    positionSetpointRotations = motor.getClosedLoopReference();
    positionErrorRotations = motor.getClosedLoopError();

    absolutePosition = cancoder.getAbsolutePosition();

    neutralControl = new NeutralOut();
    voltageControl = new VoltageOut(0.0);
    voltagePositionControl = new MotionMagicVoltage(0.0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        positionRotations,
        velocityRotationsPerSecond,
        appliedVolts,
        currentAmps,
        temperatureCelcius,
        positionSetpointRotations,
        positionErrorRotations,
        absolutePosition);

    motor.optimizeBusUtilization(50.0, 1.0);
    cancoder.optimizeBusUtilization(50.0, 1.0);

    motor.setPosition(((absolutePosition.getValueAsDouble() * WhiplashArmConstants.GEAR_RATIO)));
  }

  @Override
  public void updateInputs(WhiplashArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        positionRotations,
        velocityRotationsPerSecond,
        appliedVolts,
        currentAmps,
        temperatureCelcius,
        positionSetpointRotations,
        positionErrorRotations,
        absolutePosition);
    positionSetpointRotations.refresh();
    positionErrorRotations.refresh();

    inputs.position = Rotation2d.fromRotations(positionRotations.getValueAsDouble());
    inputs.velocityRadiansPerSecond =
        Units.rotationsToRadians(velocityRotationsPerSecond.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.temperatureCelsius = temperatureCelcius.getValueAsDouble();

    inputs.absolutePosition = Rotation2d.fromRotations(absolutePosition.getValueAsDouble());
    inputs.positionGoal = positionGoal;
    inputs.positionSetpoint =
        Rotation2d.fromRotations(positionSetpointRotations.getValueAsDouble());
    inputs.positionError = Rotation2d.fromRotations(positionErrorRotations.getValueAsDouble());
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageControl.withOutput(volts).withEnableFOC(false));
  }

  @Override
  public void setPosition(Rotation2d setpointPosition) {
    positionGoal = setpointPosition;
    motor.setControl(
        voltagePositionControl
            .withPosition(positionGoal.getRotations())
            .withUpdateFreqHz(1000)
            .withEnableFOC(true));
  }

  @Override
  public void setPID(double kp, double ki, double kd) {
    motorConfig.Slot0.kP = kp;
    motorConfig.Slot0.kI = ki;
    motorConfig.Slot0.kD = kd;
    motor.getConfigurator().apply(motorConfig);
  }

  @Override
  public void setFeedforward(double ks, double kg, double kv) {
    motorConfig.Slot0.kS = ks;
    motorConfig.Slot0.kV = kv;
    motorConfig.Slot0.kG = kg;
    motor.getConfigurator().apply(motorConfig);
  }

  @Override
  public void setProfile(
      double maxVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared,
      double goalToleranceRadians) {
    motorConfig.MotionMagic.MotionMagicCruiseVelocity =
        Units.radiansToRotations(maxVelocityRadiansPerSecond);
    motorConfig.MotionMagic.MotionMagicAcceleration =
        Units.radiansToRotations(maxAccelerationRadiansPerSecondSquared);
    motor.getConfigurator().apply(motorConfig);
  }

  @Override
  public boolean atSetpoint() {
    return Math.abs(positionGoal.getRotations() - positionRotations.getValueAsDouble())
        <= Units.radiansToRotations(WhiplashArmConstants.CONSTRAINTS.goalToleranceRadians().get());
  }

  @Override
  public void stop() {
    motor.setControl(neutralControl);
  }
}
