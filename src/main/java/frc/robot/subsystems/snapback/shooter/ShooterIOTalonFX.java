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

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX leftFlywheel;
  public StatusSignal<Angle> leftPosition;
  public StatusSignal<AngularVelocity> leftVelocityRotationsPerSecond;
  public StatusSignal<Double> leftVelocitySetpointRadiansPerSecond;
  public StatusSignal<Double> leftVelocityErrorRadiansPerSecond;
  public StatusSignal<Voltage> leftAppliedVolts;
  public StatusSignal<Current> leftCurrentAmps;
  public StatusSignal<Temperature> leftTemperatureCelsius;

  private final TalonFX rightFlywheel;
  public StatusSignal<Angle> rightPosition;
  public StatusSignal<AngularVelocity> rightVelocityRotationsPerSecond;
  public StatusSignal<Double> rightVelocitySetpointRadiansPerSecond;
  public StatusSignal<Double> rightVelocityErrorRadiansPerSecond;
  public StatusSignal<Voltage> rightAppliedVolts;
  public StatusSignal<Current> rightCurrentAmps;
  public StatusSignal<Temperature> rightTemperatureCelsius;

  private final TalonFX accelerator;
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

  private VoltageOut voltageControl;
  private MotionMagicVelocityTorqueCurrentFOC velocityControl;

  public ShooterIOTalonFX() {
    // Left flywheel is the right one when looking at the robot from the front (shooter side); right
    // flywheel is the left one
    leftFlywheel = new TalonFX(ShooterConstants.LEFT_FLYWHEEL_MOTOR_CAN_ID);
    rightFlywheel = new TalonFX(ShooterConstants.RIGHT_FLYWHEEL_MOTOR_CAN_ID);
    accelerator = new TalonFX(ShooterConstants.ACCELERATOR_MOTOR_CAN_ID);

    TalonFXConfiguration rightFlywheelConfig = new TalonFXConfiguration();
    rightFlywheelConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
    rightFlywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rightFlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rightFlywheelConfig.Feedback.SensorToMechanismRatio = ShooterConstants.FLYWHEEL_GEAR_REDUCTION;
    rightFlywheelConfig.Slot0.kP = ShooterConstants.GAINS.kp();
    rightFlywheelConfig.Slot0.kI = ShooterConstants.GAINS.ki();
    rightFlywheelConfig.Slot0.kD = ShooterConstants.GAINS.kd();
    rightFlywheelConfig.Slot0.kS = ShooterConstants.GAINS.ks();
    rightFlywheelConfig.Slot0.kV = ShooterConstants.GAINS.kv();
    rightFlywheelConfig.Slot0.kA = ShooterConstants.GAINS.ka();
    rightFlywheel.getConfigurator().apply(rightFlywheelConfig);

    TalonFXConfiguration leftFlywheelConfig = rightFlywheelConfig;
    leftFlywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftFlywheel.getConfigurator().apply(leftFlywheelConfig);

    TalonFXConfiguration acceleratorConfig = rightFlywheelConfig;
    acceleratorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    acceleratorConfig.Feedback.SensorToMechanismRatio = ShooterConstants.ACCELERATOR_GEAR_REDUCTION;
    accelerator.getConfigurator().apply(acceleratorConfig);

    leftPosition = leftFlywheel.getPosition();
    leftVelocityRotationsPerSecond = leftFlywheel.getVelocity();
    leftVelocitySetpointRadiansPerSecond = leftFlywheel.getClosedLoopReference();
    leftVelocityErrorRadiansPerSecond = leftFlywheel.getClosedLoopError();
    leftAppliedVolts = leftFlywheel.getMotorVoltage();
    leftCurrentAmps = leftFlywheel.getSupplyCurrent();
    leftTemperatureCelsius = leftFlywheel.getDeviceTemp();

    rightPosition = rightFlywheel.getPosition();
    rightVelocityRotationsPerSecond = rightFlywheel.getVelocity();
    rightVelocitySetpointRadiansPerSecond = rightFlywheel.getClosedLoopReference();
    rightVelocityErrorRadiansPerSecond = rightFlywheel.getClosedLoopError();
    rightAppliedVolts = rightFlywheel.getMotorVoltage();
    rightCurrentAmps = rightFlywheel.getSupplyCurrent();
    rightTemperatureCelsius = rightFlywheel.getDeviceTemp();

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
        leftVelocitySetpointRadiansPerSecond,
        leftVelocityErrorRadiansPerSecond,
        rightPosition,
        rightVelocityRotationsPerSecond,
        rightAppliedVolts,
        rightCurrentAmps,
        rightTemperatureCelsius,
        rightVelocitySetpointRadiansPerSecond,
        rightVelocityErrorRadiansPerSecond,
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
  public void updateInputs(ShooterIOInputs inputs) {
    boolean leftConnected =
        BaseStatusSignal.refreshAll(
                leftPosition,
                leftVelocityRotationsPerSecond,
                leftVelocitySetpointRadiansPerSecond,
                leftVelocityErrorRadiansPerSecond,
                leftAppliedVolts,
                leftCurrentAmps,
                leftTemperatureCelsius)
            .isOK();
    boolean rightConnected =
        BaseStatusSignal.refreshAll(
                rightPosition,
                rightVelocityRotationsPerSecond,
                rightVelocitySetpointRadiansPerSecond,
                rightVelocityErrorRadiansPerSecond,
                rightAppliedVolts,
                rightCurrentAmps,
                rightTemperatureCelsius)
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
    inputs.leftVelocitySetpointRadiansPerSecond =
        Units.rotationsToRadians(leftVelocitySetpointRadiansPerSecond.getValueAsDouble());

    inputs.leftVelocityErrorRadiansPerSecond =
        Units.rotationsToRadians(leftVelocityErrorRadiansPerSecond.getValueAsDouble());
    inputs.leftVelocityRadiansPerSecond =
        Units.rotationsToRadians(leftVelocityRotationsPerSecond.getValueAsDouble());
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftCurrentAmps = leftCurrentAmps.getValueAsDouble();
    inputs.leftTemperatureCelsius = leftTemperatureCelsius.getValueAsDouble();

    inputs.rightPosition = Rotation2d.fromRotations(rightPosition.getValueAsDouble());
    inputs.rightVelocitySetpointRadiansPerSecond =
        Units.rotationsToRadians(rightVelocitySetpointRadiansPerSecond.getValueAsDouble());
    inputs.rightVelocityErrorRadiansPerSecond =
        Units.rotationsToRadians(rightVelocityErrorRadiansPerSecond.getValueAsDouble());
    inputs.rightVelocityRadiansPerSecond =
        Units.rotationsToRadians(rightVelocityRotationsPerSecond.getValueAsDouble());
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightCurrentAmps = rightCurrentAmps.getValueAsDouble();
    inputs.rightTemperatureCelsius = rightTemperatureCelsius.getValueAsDouble();

    inputs.acceleratorPosition = Rotation2d.fromRotations(acceleratorPosition.getValueAsDouble());
    inputs.acceleratorVelocityRadiansPerSecond =
        Units.rotationsToRadians(acceleratorVelocityRotationsPerSecond.getValueAsDouble());
    inputs.acceleratorAppliedVolts = acceleratorAppliedVolts.getValueAsDouble();
    inputs.acceleratorCurrentAmps = acceleratorCurrentAmps.getValueAsDouble();
    inputs.acceleratorTemperatureCelsius = acceleratorTemperatureCelsius.getValueAsDouble();
  }

  @Override
  public void setLeftVoltage(double volts) {
    leftFlywheel.setControl(voltageControl.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setRightVoltage(double volts) {
    rightFlywheel.setControl(voltageControl.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setAcceleratorVoltage(double volts) {
    accelerator.setControl(voltageControl.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setLeftVelocityGoal(double velocityRadiansPerSecond) {
    leftFlywheel.setControl(
        velocityControl.withVelocity(velocityRadiansPerSecond).withEnableFOC(true));
  }

  @Override
  public void setRightVelocitySetpoint(double velocityRadiansPerSecond) {
    rightFlywheel.setControl(
        velocityControl.withVelocity(velocityRadiansPerSecond).withEnableFOC(true));
  }
}
