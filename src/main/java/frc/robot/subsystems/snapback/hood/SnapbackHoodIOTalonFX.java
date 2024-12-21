package frc.robot.subsystems.snapback.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

public class SnapbackHoodIOTalonFX implements SnapbackHoodIO {
  private final TalonFX hoodMotor;

  public StatusSignal<Angle> positionRotations;
  public StatusSignal<AngularVelocity> velocityRotationsPerSecond;
  public StatusSignal<Current> currentAmps;
  public StatusSignal<Temperature> tempratureCelsius;
  public Rotation2d positionGoal;
  public StatusSignal<Double> positionSetpointRotations;
  public StatusSignal<Double> positionErrorRotations;

  private VoltageOut voltageControlRequest;
  private MotionMagicVoltage positionControlRequest;

  public SnapbackHoodIOTalonFX() {
    hoodMotor = new TalonFX(SnapbackHoodConstants.MOTOR_CAN_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = SnapbackHoodConstants.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = SnapbackHoodConstants.GEAR_RATIO;
    config.Slot0.kP = SnapbackHoodConstants.GAINS.kp().get();
    config.Slot0.kD = SnapbackHoodConstants.GAINS.kd().get();
    config.Slot0.kS = SnapbackHoodConstants.GAINS.ks().get();
    config.Slot0.kV = SnapbackHoodConstants.GAINS.kv().get();
    config.Slot0.kA = SnapbackHoodConstants.GAINS.ka().get();
    config.MotionMagic.MotionMagicCruiseVelocity =
        SnapbackHoodConstants.CONSTRAINTS.maxVelocityRadiansPerSecond().get();
    config.MotionMagic.MotionMagicAcceleration =
        SnapbackHoodConstants.CONSTRAINTS.maxAccelerationRadiansPerSecondSqaured().get();
    hoodMotor.getConfigurator().apply(config);

    positionRotations = hoodMotor.getPosition();
    velocityRotationsPerSecond = hoodMotor.getVelocity();
    currentAmps = hoodMotor.getSupplyCurrent();
    tempratureCelsius = hoodMotor.getDeviceTemp();
    positionSetpointRotations = hoodMotor.getClosedLoopReference();
    positionErrorRotations = hoodMotor.getClosedLoopError();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        positionRotations,
        velocityRotationsPerSecond,
        currentAmps,
        tempratureCelsius,
        positionSetpointRotations,
        positionErrorRotations);
    hoodMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(SnapbackHoodIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        positionRotations,
        velocityRotationsPerSecond,
        currentAmps,
        tempratureCelsius,
        positionSetpointRotations,
        positionErrorRotations);

    inputs.position = Rotation2d.fromRotations(positionRotations.getValueAsDouble());
    inputs.velocityRadiansPerSecond =
        Units.rotationsToRadians(velocityRotationsPerSecond.getValueAsDouble());
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.temperatureCelsius = tempratureCelsius.getValueAsDouble();
    inputs.positionGoal = positionGoal;
    inputs.positionSetpoint =
        Rotation2d.fromRotations(positionSetpointRotations.getValueAsDouble());
    inputs.positionError = Rotation2d.fromRotations(positionErrorRotations.getValueAsDouble());
  }

  @Override
  public void setVoltage(double volts) {
    hoodMotor.setControl(voltageControlRequest.withOutput(volts).withUpdateFreqHz(1000.0));
  }

  @Override
  public void setPosition(Rotation2d position) {
    positionGoal = position;
    hoodMotor.setControl(
        positionControlRequest.withPosition(positionGoal.getRotations()).withUpdateFreqHz(1000.0));
  }

  @Override
  public void setPID(double kp, double ki, double kd) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = kp;
    config.Slot0.kI = ki;
    config.Slot0.kD = kd;
    hoodMotor.getConfigurator().apply(config);
  }

  @Override
  public void setFeedforward(double ks, double kv, double ka) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kS = ks;
    config.Slot0.kV = kv;
    config.Slot0.kA = ka;
    hoodMotor.getConfigurator().apply(config);
  }

  @Override
  public void setProfile(
      double maxVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared,
      double goalToleranceRadians) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotionMagic.MotionMagicCruiseVelocity = maxVelocityRadiansPerSecond;
    config.MotionMagic.MotionMagicAcceleration = maxAccelerationRadiansPerSecondSquared;
    hoodMotor.getConfigurator().apply(config);
  }

  @Override
  public boolean atGoal() {
    return Math.abs(positionGoal.getRotations() - positionRotations.getValueAsDouble())
        <= SnapbackHoodConstants.CONSTRAINTS.goalToleranceRadians().get();
  }
}
