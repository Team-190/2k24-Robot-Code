package frc.robot.subsystems.snapback.shooter;

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

public class SnapbackShooterIOTalonFX implements SnapbackShooterIO {
  private final TalonFX leftFlywheel;
  private final TalonFX rightFlywheel;
  private final TalonFX accelerator;

  public StatusSignal<Angle> leftPosition;
  public StatusSignal<AngularVelocity> leftVelocityRotationsPerSecond;
  public StatusSignal<Voltage> leftAppliedVolts;
  public StatusSignal<Current> leftCurrentAmps;
  public StatusSignal<Temperature> leftTemperatureCelsius;
  public double leftVelocityGoalRadiansPerSecond;
  public StatusSignal<Double> leftVelocitySetpointRotationsPerSecond;
  public StatusSignal<Double> leftVelocityErrorRotationsPerSecond;

  public StatusSignal<Angle> rightPosition;
  public StatusSignal<AngularVelocity> rightVelocityRotationsPerSecond;
  public StatusSignal<Voltage> rightAppliedVolts;
  public StatusSignal<Current> rightCurrentAmps;
  public StatusSignal<Temperature> rightTemperatureCelsius;
  public double rightVelocityGoalRadiansPerSecond;
  public StatusSignal<Double> rightVelocitySetpointRotationsPerSecond;
  public StatusSignal<Double> rightVelocityErrorRotationsPerSecond;

  public StatusSignal<Angle> acceleratorPosition;
  public StatusSignal<AngularVelocity> acceleratorVelocityRotationsPerSecond;
  public StatusSignal<Voltage> acceleratorAppliedVolts;
  public StatusSignal<Current> acceleratorCurrentAmps;
  public StatusSignal<Temperature> acceleratorTemperatureCelsius;

  private final Alert leftDisconnectedAlert =
      new Alert("Shooter left Talon is disconnected, check CAN bus.", AlertType.ERROR);
  private final Alert rightDisconnectedAlert =
      new Alert("Shooter right Talon is disconnected, check CAN bus.", AlertType.ERROR);
  private final Alert acceleratorDisconnectedAlert =
      new Alert("Shooter accelerator Talon is disconnected, check CAN bus.", AlertType.ERROR);

  private VoltageOut voltageControlRequest;
  private MotionMagicVelocityTorqueCurrentFOC velocityControlRequest;

  public SnapbackShooterIOTalonFX() {
    // Left flywheel is the right one when looking at the robot from the front
    // (shooter side); right
    // flywheel is the left one
    leftFlywheel = new TalonFX(SnapbackShooterConstants.LEFT_FLYWHEEL_MOTOR_CAN_ID);
    rightFlywheel = new TalonFX(SnapbackShooterConstants.RIGHT_FLYWHEEL_MOTOR_CAN_ID);
    accelerator = new TalonFX(SnapbackShooterConstants.ACCELERATOR_MOTOR_CAN_ID);

    TalonFXConfiguration rightFlywheelConfig = new TalonFXConfiguration();
    rightFlywheelConfig.CurrentLimits.SupplyCurrentLimit =
        SnapbackShooterConstants.FLYWHEEL_CURRENT_LIMIT;
    rightFlywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rightFlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rightFlywheelConfig.Feedback.SensorToMechanismRatio =
        SnapbackShooterConstants.FLYWHEEL_GEAR_REDUCTION;
    rightFlywheelConfig.Slot0.kP = SnapbackShooterConstants.GAINS.kp().get();
    rightFlywheelConfig.Slot0.kI = SnapbackShooterConstants.GAINS.ki().get();
    rightFlywheelConfig.Slot0.kD = SnapbackShooterConstants.GAINS.kd().get();
    rightFlywheelConfig.Slot0.kS = SnapbackShooterConstants.GAINS.ks();
    rightFlywheelConfig.Slot0.kV = SnapbackShooterConstants.GAINS.kv();
    rightFlywheelConfig.Slot0.kA = SnapbackShooterConstants.GAINS.ka();
    rightFlywheelConfig.MotionMagic.MotionMagicCruiseVelocity =
        SnapbackShooterConstants.CRUISE_VELOCITY.get();
    rightFlywheelConfig.MotionMagic.MotionMagicAcceleration =
        SnapbackShooterConstants.MAX_ACCELERATION.get();
    rightFlywheel.getConfigurator().apply(rightFlywheelConfig);

    TalonFXConfiguration leftFlywheelConfig = rightFlywheelConfig;
    leftFlywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftFlywheel.getConfigurator().apply(leftFlywheelConfig);

    TalonFXConfiguration acceleratorConfig = rightFlywheelConfig;
    acceleratorConfig.CurrentLimits.SupplyCurrentLimit =
        SnapbackShooterConstants.ACCELERATOR_CURRENT_LIMIT;
    acceleratorConfig.Feedback.SensorToMechanismRatio =
        SnapbackShooterConstants.ACCELERATOR_GEAR_REDUCTION;
    accelerator.getConfigurator().apply(acceleratorConfig);

    leftPosition = leftFlywheel.getPosition();
    leftVelocityRotationsPerSecond = leftFlywheel.getVelocity();
    leftAppliedVolts = leftFlywheel.getMotorVoltage();
    leftCurrentAmps = leftFlywheel.getSupplyCurrent();
    leftTemperatureCelsius = leftFlywheel.getDeviceTemp();

    leftVelocitySetpointRotationsPerSecond = leftFlywheel.getClosedLoopReference();
    leftVelocityErrorRotationsPerSecond = leftFlywheel.getClosedLoopError();

    rightPosition = rightFlywheel.getPosition();
    rightVelocityRotationsPerSecond = rightFlywheel.getVelocity();
    rightAppliedVolts = rightFlywheel.getMotorVoltage();
    rightCurrentAmps = rightFlywheel.getSupplyCurrent();
    rightTemperatureCelsius = rightFlywheel.getDeviceTemp();

    rightVelocitySetpointRotationsPerSecond = rightFlywheel.getClosedLoopReference();
    rightVelocityErrorRotationsPerSecond = rightFlywheel.getClosedLoopError();

    acceleratorPosition = accelerator.getPosition();
    acceleratorVelocityRotationsPerSecond = accelerator.getVelocity();
    acceleratorAppliedVolts = accelerator.getMotorVoltage();
    acceleratorCurrentAmps = accelerator.getSupplyCurrent();
    acceleratorTemperatureCelsius = accelerator.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        leftPosition,
        leftVelocityRotationsPerSecond,
        leftAppliedVolts,
        leftCurrentAmps,
        leftTemperatureCelsius,
        leftVelocitySetpointRotationsPerSecond,
        leftVelocityErrorRotationsPerSecond,
        rightPosition,
        rightVelocityRotationsPerSecond,
        rightAppliedVolts,
        rightCurrentAmps,
        rightTemperatureCelsius,
        rightVelocitySetpointRotationsPerSecond,
        rightVelocityErrorRotationsPerSecond,
        acceleratorPosition,
        acceleratorVelocityRotationsPerSecond,
        acceleratorAppliedVolts,
        acceleratorCurrentAmps,
        acceleratorTemperatureCelsius);

    leftFlywheel.optimizeBusUtilization();
    rightFlywheel.optimizeBusUtilization();
    accelerator.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(SnapbackShooterIOInputs inputs) {
    boolean leftConnected =
        BaseStatusSignal.refreshAll(
                leftPosition,
                leftVelocityRotationsPerSecond,
                leftAppliedVolts,
                leftCurrentAmps,
                leftTemperatureCelsius,
                leftVelocitySetpointRotationsPerSecond,
                leftVelocityErrorRotationsPerSecond)
            .isOK();
    boolean rightConnected =
        BaseStatusSignal.refreshAll(
                rightPosition,
                rightVelocityRotationsPerSecond,
                rightAppliedVolts,
                rightCurrentAmps,
                rightTemperatureCelsius,
                rightVelocitySetpointRotationsPerSecond,
                rightVelocityErrorRotationsPerSecond)
            .isOK();
    boolean acceleratorConnected =
        BaseStatusSignal.refreshAll(
                acceleratorPosition,
                acceleratorVelocityRotationsPerSecond,
                acceleratorAppliedVolts,
                acceleratorCurrentAmps,
                acceleratorTemperatureCelsius)
            .isOK();
    leftDisconnectedAlert.set(!leftConnected);
    rightDisconnectedAlert.set(!rightConnected);
    acceleratorDisconnectedAlert.set(!acceleratorConnected);

    inputs.leftPosition = Rotation2d.fromRotations(leftPosition.getValueAsDouble());
    inputs.leftVelocityRadiansPerSecond =
        Units.rotationsToRadians(leftVelocityRotationsPerSecond.getValueAsDouble());
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftCurrentAmps = leftCurrentAmps.getValueAsDouble();
    inputs.leftTemperatureCelsius = leftTemperatureCelsius.getValueAsDouble();
    inputs.leftVelocityGoalRadiansPerSecond = leftVelocityGoalRadiansPerSecond;
    inputs.leftVelocitySetpointRadiansPerSecond =
        Units.rotationsToRadians(leftVelocitySetpointRotationsPerSecond.getValueAsDouble());
    inputs.leftVelocityErrorRadiansPerSecond =
        Units.rotationsToRadians(leftVelocityErrorRotationsPerSecond.getValueAsDouble());

    inputs.rightPosition = Rotation2d.fromRotations(rightPosition.getValueAsDouble());
    inputs.rightVelocityRadiansPerSecond =
        Units.rotationsToRadians(rightVelocityRotationsPerSecond.getValueAsDouble());
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightCurrentAmps = rightCurrentAmps.getValueAsDouble();
    inputs.rightTemperatureCelsius = rightTemperatureCelsius.getValueAsDouble();
    inputs.rightVelocityGoalRadiansPerSecond = rightVelocityGoalRadiansPerSecond;
    inputs.rightVelocitySetpointRadiansPerSecond =
        Units.rotationsToRadians(rightVelocitySetpointRotationsPerSecond.getValueAsDouble());
    inputs.rightVelocityErrorRadiansPerSecond =
        Units.rotationsToRadians(rightVelocityErrorRotationsPerSecond.getValueAsDouble());

    inputs.acceleratorPosition = Rotation2d.fromRotations(acceleratorPosition.getValueAsDouble());
    inputs.acceleratorVelocityRadiansPerSecond =
        Units.rotationsToRadians(acceleratorVelocityRotationsPerSecond.getValueAsDouble());
    inputs.acceleratorAppliedVolts = acceleratorAppliedVolts.getValueAsDouble();
    inputs.acceleratorCurrentAmps = acceleratorCurrentAmps.getValueAsDouble();
    inputs.acceleratorTemperatureCelsius = acceleratorTemperatureCelsius.getValueAsDouble();
  }

  @Override
  public void setLeftVoltage(double volts) {
    leftFlywheel.setControl(voltageControlRequest.withOutput(volts).withUpdateFreqHz(1000.0));
  }

  @Override
  public void setRightVoltage(double volts) {
    rightFlywheel.setControl(voltageControlRequest.withOutput(volts).withUpdateFreqHz(1000.0));
  }

  @Override
  public void setAcceleratorVoltage(double volts) {
    accelerator.setControl(voltageControlRequest.withOutput(volts).withUpdateFreqHz(1000.0));
  }

  @Override
  public void setLeftVelocityGoal(double velocityRadiansPerSecond) {
    leftVelocityGoalRadiansPerSecond = velocityRadiansPerSecond;
    leftFlywheel.setControl(
        velocityControlRequest.withVelocity(velocityRadiansPerSecond).withUpdateFreqHz(1000.0));
  }

  @Override
  public void setRightVelocityGoal(double velocityRadiansPerSecond) {
    rightVelocityGoalRadiansPerSecond = velocityRadiansPerSecond;
    rightFlywheel.setControl(
        velocityControlRequest.withVelocity(velocityRadiansPerSecond).withUpdateFreqHz(1000.0));
  }

  @Override
  public boolean atGoal() {
    return Math.abs(
                leftVelocitySetpointRotationsPerSecond.getValueAsDouble()
                    - Units.rotationsToRadians(leftVelocityRotationsPerSecond.getValueAsDouble()))
            < SnapbackShooterConstants.FLYWHEEL_TOLERANCE_RAD_PER_SEC
        && Math.abs(
                rightVelocitySetpointRotationsPerSecond.getValueAsDouble()
                    - Units.rotationsToRadians(rightVelocityRotationsPerSecond.getValueAsDouble()))
            < SnapbackShooterConstants.FLYWHEEL_TOLERANCE_RAD_PER_SEC;
  }
}
