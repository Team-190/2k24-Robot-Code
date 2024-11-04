package frc.robot.subsystems.snapback.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
  public StatusSignal<AngularVelocity> leftVelocityRadiansPerSecond;
  public StatusSignal<Voltage> leftAppliedVolts;
  public StatusSignal<Current> leftCurrentAmps;
  public StatusSignal<Temperature> leftTemperatureCelsius;

  private final TalonFX rightFlywheel;
  public StatusSignal<Angle> rightPosition;
  public StatusSignal<AngularVelocity> rightVelocityRadiansPerSecond;
  public StatusSignal<Voltage> rightAppliedVolts;
  public StatusSignal<Current> rightCurrentAmps;
  public StatusSignal<Temperature> rightTemperatureCelsius;

  private final TalonFX accelerator;
  public StatusSignal<Angle> acceleratorPosition;
  public StatusSignal<AngularVelocity> acceleratorVelocityRadiansPerSecond;
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

  public ShooterIOTalonFX() {
    leftFlywheel = new TalonFX(ShooterConstants.LEFT_FLYWHEEL_MOTOR_CAN_ID);
    rightFlywheel = new TalonFX(ShooterConstants.RIGHT_FLYWHEEL_MOTOR_CAN_ID);
    accelerator = new TalonFX(ShooterConstants.ACCELERATOR_MOTOR_CAN_ID);

    TalonFXConfiguration rightFlywheelConfig = new TalonFXConfiguration();
    rightFlywheelConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
    rightFlywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rightFlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rightFlywheelConfig.Feedback.SensorToMechanismRatio = ShooterConstants.FLYWHEEL_GEAR_RATIO;
    rightFlywheel.getConfigurator().apply(rightFlywheelConfig);

    TalonFXConfiguration leftFlywheelConfig = rightFlywheelConfig;
    leftFlywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftFlywheel.getConfigurator().apply(leftFlywheelConfig);

    TalonFXConfiguration acceleratorConfig = rightFlywheelConfig;
    acceleratorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    acceleratorConfig.Feedback.SensorToMechanismRatio = ShooterConstants.ACCELERATOR_GEAR_RATIO;
    accelerator.getConfigurator().apply(acceleratorConfig);

    leftPosition = leftFlywheel.getPosition();
    leftVelocityRadiansPerSecond = leftFlywheel.getVelocity();
    leftAppliedVolts = leftFlywheel.getMotorVoltage();
    leftCurrentAmps = leftFlywheel.getSupplyCurrent();
    leftTemperatureCelsius = leftFlywheel.getDeviceTemp();

    rightPosition = rightFlywheel.getPosition();
    rightVelocityRadiansPerSecond = rightFlywheel.getVelocity();
    rightAppliedVolts = rightFlywheel.getMotorVoltage();
    rightCurrentAmps = rightFlywheel.getSupplyCurrent();
    rightTemperatureCelsius = rightFlywheel.getDeviceTemp();

    acceleratorPosition = accelerator.getPosition();
    acceleratorVelocityRadiansPerSecond = accelerator.getVelocity();
    acceleratorAppliedVolts = accelerator.getMotorVoltage();
    acceleratorCurrentAmps = accelerator.getSupplyCurrent();
    acceleratorTemperatureCelsius = accelerator.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        leftPosition,
        leftVelocityRadiansPerSecond,
        leftAppliedVolts,
        leftCurrentAmps,
        leftTemperatureCelsius,
        rightPosition,
        rightVelocityRadiansPerSecond,
        rightAppliedVolts,
        rightCurrentAmps,
        rightTemperatureCelsius,
        acceleratorPosition,
        acceleratorVelocityRadiansPerSecond,
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
                leftVelocityRadiansPerSecond,
                leftAppliedVolts,
                leftCurrentAmps,
                leftTemperatureCelsius)
            .isOK();
    boolean rightConnected =
        BaseStatusSignal.refreshAll(
                rightPosition,
                rightVelocityRadiansPerSecond,
                rightAppliedVolts,
                rightCurrentAmps,
                rightTemperatureCelsius)
            .isOK();
    boolean acceleratorConnected =
        BaseStatusSignal.refreshAll(
                acceleratorPosition,
                acceleratorVelocityRadiansPerSecond,
                acceleratorAppliedVolts,
                acceleratorCurrentAmps,
                acceleratorTemperatureCelsius)
            .isOK();
    leftDisconnectedAlert.set(!leftConnected);
    rightDisconnectedAlert.set(!rightConnected);
    acceleratorDisconnectedAlert.set(!acceleratorConnected);

    inputs.leftPosition = Rotation2d.fromRotations(leftPosition.getValueAsDouble());
    inputs.leftVelocityRadiansPerSecond =
        Units.rotationsToRadians(leftVelocityRadiansPerSecond.getValueAsDouble());
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftCurrentAmps = leftCurrentAmps.getValueAsDouble();
    inputs.leftTemperatureCelsius = leftTemperatureCelsius.getValueAsDouble();

    inputs.rightPosition = Rotation2d.fromRotations(rightPosition.getValueAsDouble());
    inputs.rightVelocityRadiansPerSecond =
        Units.rotationsToRadians(rightVelocityRadiansPerSecond.getValueAsDouble());
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightCurrentAmps = rightCurrentAmps.getValueAsDouble();
    inputs.rightTemperatureCelsius = rightTemperatureCelsius.getValueAsDouble();

    inputs.acceleratorPosition = Rotation2d.fromRotations(acceleratorPosition.getValueAsDouble());
    inputs.acceleratorVelocityRadiansPerSecond =
        Units.rotationsToRadians(acceleratorVelocityRadiansPerSecond.getValueAsDouble());
    inputs.acceleratorAppliedVolts = acceleratorAppliedVolts.getValueAsDouble();
    inputs.acceleratorCurrentAmps = acceleratorCurrentAmps.getValueAsDouble();
    inputs.acceleratorTemperatureCelsius = acceleratorTemperatureCelsius.getValueAsDouble();
  }

  @Override
  public void setLeftVoltage(double volts) {
    leftFlywheel.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void setRightVoltage(double volts) {
    rightFlywheel.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void setAcceleratorVoltage(double volts) {
    accelerator.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void setLeftVelocitySetpoint(double velocityRadiansPerSecond) {}
}
