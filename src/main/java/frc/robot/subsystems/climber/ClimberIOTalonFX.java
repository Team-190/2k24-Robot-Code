package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX leftTalon;
  private final TalonFX rightTalon;
  private final Solenoid leftLock;
  private final Solenoid rightLock;

  private final StatusSignal<Double> leftPosition;
  private final StatusSignal<Double> leftVelocity;
  private final StatusSignal<Double> leftAppliedVolts;
  private final StatusSignal<Double> leftCurrentAmps;
  private final StatusSignal<Double> leftTempCelcius;

  private final StatusSignal<Double> rightPosition;
  private final StatusSignal<Double> rightVelocity;
  private final StatusSignal<Double> rightAppliedVolts;
  private final StatusSignal<Double> rightCurrentAmps;
  private final StatusSignal<Double> rightTempCelcius;

  private final double GEAR_RATIO = 1.0;
  private final double DRUM_CIRCUMFERENCE = Units.inchesToMeters(1.0) * Math.PI;

  private final Alert leftDisconnectedAlert =
      new Alert("Left climber Talon is disconnected, check CAN bus.", AlertType.ERROR);
  private final Alert rightDisconnectedAlert =
      new Alert("Right climber Talon is disconnected, check CAN bus.", AlertType.ERROR);

  public ClimberIOTalonFX() {
    switch (Constants.ROBOT) {
      case SNAPBACK:
        leftTalon = new TalonFX(4);
        rightTalon = new TalonFX(5);
        leftLock = new Solenoid(PneumaticsModuleType.REVPH, 4);
        rightLock = new Solenoid(PneumaticsModuleType.REVPH, 5);
        break;
      case ROBOT_2K24_TEST:
        leftTalon = new TalonFX(4);
        rightTalon = new TalonFX(5);
        leftLock = new Solenoid(PneumaticsModuleType.REVPH, 4);
        rightLock = new Solenoid(PneumaticsModuleType.REVPH, 5);
        break;
      default:
        throw new RuntimeException("Invalid robot");
    }

    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftTalon.getConfigurator().apply(config);
    rightTalon.getConfigurator().apply(config);

    leftTalon.setPosition(0.0);
    rightTalon.setPosition(0.0);

    leftPosition = leftTalon.getPosition();
    leftVelocity = leftTalon.getVelocity();
    leftAppliedVolts = leftTalon.getMotorVoltage();
    leftCurrentAmps = leftTalon.getSupplyCurrent();
    leftTempCelcius = leftTalon.getDeviceTemp();

    rightPosition = rightTalon.getPosition();
    rightVelocity = rightTalon.getVelocity();
    rightAppliedVolts = rightTalon.getMotorVoltage();
    rightCurrentAmps = rightTalon.getSupplyCurrent();
    rightTempCelcius = rightTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leftPosition,
        leftVelocity,
        leftAppliedVolts,
        leftCurrentAmps,
        leftTempCelcius,
        rightPosition,
        rightVelocity,
        rightAppliedVolts,
        rightCurrentAmps,
        rightTempCelcius);
    leftTalon.optimizeBusUtilization();
    rightTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    boolean leftConnected =
        BaseStatusSignal.refreshAll(
                leftPosition, leftVelocity, leftAppliedVolts, leftCurrentAmps, leftTempCelcius)
            .isOK();
    leftDisconnectedAlert.set(!leftConnected);

    boolean rightConnected =
        BaseStatusSignal.refreshAll(
                rightPosition, rightVelocity, rightAppliedVolts, rightCurrentAmps, rightTempCelcius)
            .isOK();
    rightDisconnectedAlert.set(!rightConnected);

    inputs.leftPositionMeters = leftPosition.getValueAsDouble() / GEAR_RATIO / DRUM_CIRCUMFERENCE;
    inputs.leftVelocityMetersPerSec =
        Units.rotationsToRadians(leftVelocity.getValueAsDouble()) / GEAR_RATIO / DRUM_CIRCUMFERENCE;
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftCurrentAmps = new double[] {leftCurrentAmps.getValueAsDouble()};
    inputs.leftTempCelcius = new double[] {leftTempCelcius.getValueAsDouble()};

    inputs.rightPositionMeters = rightPosition.getValueAsDouble() / GEAR_RATIO / DRUM_CIRCUMFERENCE;
    inputs.rightVelocityMetersPerSec =
        Units.rotationsToRadians(rightVelocity.getValueAsDouble())
            / GEAR_RATIO
            / DRUM_CIRCUMFERENCE;
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightCurrentAmps = new double[] {rightCurrentAmps.getValueAsDouble()};
    inputs.rightTempCelcius = new double[] {rightTempCelcius.getValueAsDouble()};
  }

  @Override
  public void setLeftVoltage(double volts) {
    leftTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setRightVoltage(double volts) {
    rightTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setLock(boolean isLocked) {
    leftLock.set(isLocked);
    rightLock.set(isLocked);
  }
}
