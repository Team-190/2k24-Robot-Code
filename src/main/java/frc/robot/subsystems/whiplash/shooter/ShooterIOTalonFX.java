package frc.robot.subsystems.whiplash.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX bottomFlywheel;
  private final TalonFX topFlywheel;

  public StatusSignal<Angle> bottomPosition;
  public StatusSignal<AngularVelocity> bottomVelocityRotationsPerSecond;
  public StatusSignal<Voltage> bottomAppliedVolts;
  public StatusSignal<Current> bottomCurrentAmps;
  public StatusSignal<Temperature> bottomTemperatureCelsius;
  public double bottomVelocityGoalRadiansPerSecond;
  public StatusSignal<Double> bottomVelocitySetpointRotationsPerSecond;
  public StatusSignal<Double> bottomVelocityErrorRotationsPerSecond;

  public StatusSignal<Angle> topPosition;
  public StatusSignal<AngularVelocity> topVelocityRotationsPerSecond;
  public StatusSignal<Voltage> topAppliedVolts;
  public StatusSignal<Current> topCurrentAmps;
  public StatusSignal<Temperature> topTemperatureCelsius;
  public double topVelocityGoalRadiansPerSecond;
  public StatusSignal<Double> topVelocitySetpointRotationsPerSecond;
  public StatusSignal<Double> topVelocityErrorRotationsPerSecond;

  private final Alert bottomDisconnectedAlert =
      new Alert("Shooter bottom Talon is disconnected, check CAN bus.", AlertType.ERROR);
  private final Alert topDisconnectedAlert =
      new Alert("Shooter top Talon is disconnected, check CAN bus.", AlertType.ERROR);

  private VoltageOut voltageControlRequest;
  private MotionMagicVelocityTorqueCurrentFOC velocityControlRequest;

  public ShooterIOTalonFX() {
    bottomFlywheel = new TalonFX(ShooterConstants.BOTTOM_FLYWHEEL_MOTOR_CAN_ID);
    topFlywheel = new TalonFX(ShooterConstants.TOP_FLYWHEEL_MOTOR_CAN_ID);

    TalonFXConfiguration topFlywheelConfig = new TalonFXConfiguration();
    topFlywheelConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.FLYWHEEL_CURRENT_LIMIT;
    topFlywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    topFlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    topFlywheelConfig.Feedback.SensorToMechanismRatio = ShooterConstants.FLYWHEEL_GEAR_REDUCTION;
    topFlywheelConfig.Slot0.kP = ShooterConstants.GAINS.kp().get();
    topFlywheelConfig.Slot0.kI = ShooterConstants.GAINS.ki().get();
    topFlywheelConfig.Slot0.kD = ShooterConstants.GAINS.kd().get();
    topFlywheelConfig.Slot0.kS = ShooterConstants.GAINS.ks();
    topFlywheelConfig.Slot0.kV = ShooterConstants.GAINS.kv();
    topFlywheelConfig.Slot0.kA = ShooterConstants.GAINS.ka();
    topFlywheelConfig.MotionMagic.MotionMagicCruiseVelocity =
        ShooterConstants.CRUISE_VELOCITY.get();
    topFlywheelConfig.MotionMagic.MotionMagicAcceleration = ShooterConstants.MAX_ACCELERATION.get();
    topFlywheel.getConfigurator().apply(topFlywheelConfig);

    TalonFXConfiguration bottomFlywheelConfig = topFlywheelConfig;
    bottomFlywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    bottomFlywheel.getConfigurator().apply(bottomFlywheelConfig);

    bottomPosition = bottomFlywheel.getPosition();
    bottomVelocityRotationsPerSecond = bottomFlywheel.getVelocity();
    bottomAppliedVolts = bottomFlywheel.getMotorVoltage();
    bottomCurrentAmps = bottomFlywheel.getSupplyCurrent();
    bottomTemperatureCelsius = bottomFlywheel.getDeviceTemp();

    bottomVelocitySetpointRotationsPerSecond = bottomFlywheel.getClosedLoopReference();
    bottomVelocityErrorRotationsPerSecond = bottomFlywheel.getClosedLoopError();

    topPosition = topFlywheel.getPosition();
    topVelocityRotationsPerSecond = topFlywheel.getVelocity();
    topAppliedVolts = topFlywheel.getMotorVoltage();
    topCurrentAmps = topFlywheel.getSupplyCurrent();
    topTemperatureCelsius = topFlywheel.getDeviceTemp();

    topVelocitySetpointRotationsPerSecond = topFlywheel.getClosedLoopReference();
    topVelocityErrorRotationsPerSecond = topFlywheel.getClosedLoopError();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        bottomPosition,
        bottomVelocityRotationsPerSecond,
        bottomAppliedVolts,
        bottomCurrentAmps,
        bottomTemperatureCelsius,
        bottomVelocitySetpointRotationsPerSecond,
        bottomVelocityErrorRotationsPerSecond,
        topPosition,
        topVelocityRotationsPerSecond,
        topAppliedVolts,
        topCurrentAmps,
        topTemperatureCelsius,
        topVelocitySetpointRotationsPerSecond,
        topVelocityErrorRotationsPerSecond);

    bottomFlywheel.optimizeBusUtilization();
    topFlywheel.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    boolean bottomConnected =
        BaseStatusSignal.refreshAll(
                bottomPosition,
                bottomVelocityRotationsPerSecond,
                bottomAppliedVolts,
                bottomCurrentAmps,
                bottomTemperatureCelsius,
                bottomVelocitySetpointRotationsPerSecond,
                bottomVelocityErrorRotationsPerSecond)
            .isOK();
    boolean topConnected =
        BaseStatusSignal.refreshAll(
                topPosition,
                topVelocityRotationsPerSecond,
                topAppliedVolts,
                topCurrentAmps,
                topTemperatureCelsius,
                topVelocitySetpointRotationsPerSecond,
                topVelocityErrorRotationsPerSecond)
            .isOK();
    bottomDisconnectedAlert.set(!bottomConnected);
    topDisconnectedAlert.set(!topConnected);

    inputs.bottomPosition = Rotation2d.fromRotations(bottomPosition.getValueAsDouble());
    inputs.bottomVelocityRadiansPerSecond =
        Units.rotationsToRadians(bottomVelocityRotationsPerSecond.getValueAsDouble());
    inputs.bottomAppliedVolts = bottomAppliedVolts.getValueAsDouble();
    inputs.bottomCurrentAmps = bottomCurrentAmps.getValueAsDouble();
    inputs.bottomTemperatureCelsius = bottomTemperatureCelsius.getValueAsDouble();
    inputs.bottomVelocityGoalRadiansPerSecond = bottomVelocityGoalRadiansPerSecond;
    inputs.bottomVelocitySetpointRadiansPerSecond =
        Units.rotationsToRadians(bottomVelocitySetpointRotationsPerSecond.getValueAsDouble());
    inputs.bottomVelocityErrorRadiansPerSecond =
        Units.rotationsToRadians(bottomVelocityErrorRotationsPerSecond.getValueAsDouble());

    inputs.topPosition = Rotation2d.fromRotations(topPosition.getValueAsDouble());
    inputs.topVelocityRadiansPerSecond =
        Units.rotationsToRadians(topVelocityRotationsPerSecond.getValueAsDouble());
    inputs.topAppliedVolts = topAppliedVolts.getValueAsDouble();
    inputs.topCurrentAmps = topCurrentAmps.getValueAsDouble();
    inputs.topTemperatureCelsius = topTemperatureCelsius.getValueAsDouble();
    inputs.topVelocityGoalRadiansPerSecond = topVelocityGoalRadiansPerSecond;
    inputs.topVelocitySetpointRadiansPerSecond =
        Units.rotationsToRadians(topVelocitySetpointRotationsPerSecond.getValueAsDouble());
    inputs.topVelocityErrorRadiansPerSecond =
        Units.rotationsToRadians(topVelocityErrorRotationsPerSecond.getValueAsDouble());
  }

  @Override
  public void setBottomVoltage(double volts) {
    bottomFlywheel.setControl(voltageControlRequest.withOutput(volts).withUpdateFreqHz(1000.0));
  }

  @Override
  public void setTopVoltage(double volts) {
    topFlywheel.setControl(voltageControlRequest.withOutput(volts).withUpdateFreqHz(1000.0));
  }

  @Override
  public void setBottomVelocityGoal(double velocityRadiansPerSecond) {
    bottomVelocityGoalRadiansPerSecond = velocityRadiansPerSecond;
    bottomFlywheel.setControl(
        velocityControlRequest.withVelocity(velocityRadiansPerSecond).withUpdateFreqHz(1000.0));
  }

  @Override
  public void setTopVelocityGoal(double velocityRadiansPerSecond) {
    topVelocityGoalRadiansPerSecond = velocityRadiansPerSecond;
    topFlywheel.setControl(
        velocityControlRequest.withVelocity(velocityRadiansPerSecond).withUpdateFreqHz(1000.0));
  }

  @Override
  public boolean atGoal() {
    return Math.abs(
                bottomVelocitySetpointRotationsPerSecond.getValueAsDouble()
                    - Units.rotationsToRadians(bottomVelocityRotationsPerSecond.getValueAsDouble()))
            < ShooterConstants.FLYWHEEL_TOLERANCE_RAD_PER_SEC
        && Math.abs(
                topVelocitySetpointRotationsPerSecond.getValueAsDouble()
                    - Units.rotationsToRadians(topVelocityRotationsPerSecond.getValueAsDouble()))
            < ShooterConstants.FLYWHEEL_TOLERANCE_RAD_PER_SEC;
  }
}
