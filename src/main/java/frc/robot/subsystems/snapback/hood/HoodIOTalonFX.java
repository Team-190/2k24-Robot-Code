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
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class HoodIOTalonFX implements HoodIO {
  private final TalonFX hoodMotor;
  public StatusSignal<Angle> positionRotations;
  public StatusSignal<AngularVelocity> velocityRotationsPerSecond;
  public StatusSignal<Current> currentAmps;
  public StatusSignal<Temperature> tempratureCelsius;
  public StatusSignal<Double> positionSetpointRotationsPerSecond;
  public StatusSignal<Double> positionErrorRotationsPerSecond;

  private final Alert disconnectedAlert =
      new Alert("Hood Talon is disconnected, check CAN bus.", AlertType.ERROR);

  private VoltageOut voltageControlRequest;
  private MotionMagicVoltage positionControlRequest;

  public HoodIOTalonFX() {
    hoodMotor = new TalonFX(HoodConstants.HOOD_MOTOR_CAN_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = HoodConstants.GEAR_REDUCTION;
    config.Slot0.kP = HoodConstants.GAINS.kp();
    config.Slot0.kI = HoodConstants.GAINS.ki();
    config.Slot0.kD = HoodConstants.GAINS.kd();
    config.Slot0.kS = HoodConstants.GAINS.ks();
    config.Slot0.kV = HoodConstants.GAINS.kv();
    config.Slot0.kA = HoodConstants.GAINS.ka();
    config.MotionMagic.MotionMagicAcceleration = HoodConstants.GAINS.ka();
    config.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.GAINS.kv();
    hoodMotor.getConfigurator().apply(config);

    positionRotations = hoodMotor.getPosition();
    velocityRotationsPerSecond = hoodMotor.getVelocity();
    currentAmps = hoodMotor.getSupplyCurrent();
    tempratureCelsius = hoodMotor.getDeviceTemp();
    positionSetpointRotationsPerSecond = hoodMotor.getClosedLoopOutput();
    positionErrorRotationsPerSecond = hoodMotor.getClosedLoopError();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, positionRotations, velocityRotationsPerSecond, currentAmps, tempratureCelsius);
    hoodMotor.optimizeBusUtilization();
  }

  public void updateInputs(HoodIOInputs inputs) {
    boolean isConnected =
        BaseStatusSignal.refreshAll(
                positionRotations, velocityRotationsPerSecond, currentAmps, tempratureCelsius)
            .isOK();
    disconnectedAlert.set(!isConnected);

    inputs.position = Rotation2d.fromRotations(positionRotations.getValueAsDouble());
    inputs.velocityRadiansPerSecond =
        Units.rotationsToRadians(velocityRotationsPerSecond.getValueAsDouble());
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.temperatureCelsius = tempratureCelsius.getValueAsDouble();
    inputs.positionSetpointRotationsPerSecond =
        positionSetpointRotationsPerSecond.getValueAsDouble();
    inputs.positionErrorRotationsPerSecond = positionErrorRotationsPerSecond.getValueAsDouble();
  }

  public void setVoltage(double volts) {
    hoodMotor.setControl(voltageControlRequest.withOutput(volts).withEnableFOC(true));
  }

  public void setPositionSetpoint(Rotation2d position) {
    hoodMotor.setControl(
        positionControlRequest.withPosition(position.getRotations()).withEnableFOC(true));
  }

  public void setPosition(Rotation2d position) {
    hoodMotor.setPosition(position.getRotations());
  }
}
