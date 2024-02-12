package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX rollersTalon;
  private final TalonFX intakeTalon;

  private final StatusSignal<Double> rollersPosition;
  private final StatusSignal<Double> rollersVelocity;
  private final StatusSignal<Double> rollersAppliedVolts;
  private final StatusSignal<Double> rollersCurrent;
  private final StatusSignal<Double> rollersTemperature;

  private final StatusSignal<Double> intakePosition;
  private final StatusSignal<Double> intakeVelocity;
  private final StatusSignal<Double> intakeAppliedVolts;
  private final StatusSignal<Double> intakeCurrent;
  private final StatusSignal<Double> intakeTemperature;

  private final double ROLLERS_GEAR_RATIO = 1.6;
  private final double INTAKE_GEAR_RATIO = 1.0;

  private final Alert rollersDisconnectedAlert =
      new Alert("Rollers Talon is disconnected, check CAN bus.", AlertType.ERROR);

  private final Alert intakeDisconnectedAlert =
      new Alert("Intake Talon is disconnected, check CAN bus.", AlertType.ERROR);

  public IntakeIOTalonFX() {
    switch (Constants.ROBOT) {
      case ROBOT_2K24_C:
        rollersTalon = new TalonFX(40);
        intakeTalon = new TalonFX(41);
        break;
      case ROBOT_2K24_P:
        rollersTalon = new TalonFX(40);
        intakeTalon = new TalonFX(41);
        break;
      case ROBOT_2K24_TEST:
        rollersTalon = new TalonFX(40);
        intakeTalon = new TalonFX(41);
        break;
      default:
        throw new RuntimeException("Invalid robot");
    }

    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollersTalon.getConfigurator().apply(config);

    rollersPosition = rollersTalon.getPosition();
    rollersVelocity = rollersTalon.getVelocity();
    rollersAppliedVolts = rollersTalon.getMotorVoltage();
    rollersCurrent = rollersTalon.getSupplyCurrent();
    rollersTemperature = rollersTalon.getDeviceTemp();

    intakePosition = intakeTalon.getPosition();
    intakeVelocity = intakeTalon.getVelocity();
    intakeAppliedVolts = intakeTalon.getMotorVoltage();
    intakeCurrent = intakeTalon.getSupplyCurrent();
    intakeTemperature = intakeTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, rollersVelocity, intakeVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        rollersPosition,
        rollersAppliedVolts,
        rollersCurrent,
        rollersTemperature,
        intakePosition,
        intakeAppliedVolts,
        intakeCurrent,
        intakeTemperature);
    rollersTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    boolean rollersConnected =
        BaseStatusSignal.refreshAll(
                rollersVelocity,
                rollersPosition,
                rollersAppliedVolts,
                rollersCurrent,
                rollersTemperature)
            .isOK();
    rollersDisconnectedAlert.set(!rollersConnected);

    boolean intakeConnected =
        BaseStatusSignal.refreshAll(
                intakeVelocity,
                intakePosition,
                intakeAppliedVolts,
                intakeCurrent,
                intakeTemperature)
            .isOK();
    intakeDisconnectedAlert.set(!intakeConnected);

    inputs.rollersPositionRad =
        Units.rotationsToRadians(rollersPosition.getValueAsDouble()) / ROLLERS_GEAR_RATIO;
    inputs.rollersVelocityRadPerSec =
        Units.rotationsToRadians(rollersVelocity.getValueAsDouble()) / ROLLERS_GEAR_RATIO;
    inputs.rollersAppliedVolts = rollersAppliedVolts.getValueAsDouble();
    inputs.rollersCurrentAmps = new double[] {rollersCurrent.getValueAsDouble()};
    inputs.rollersTempCelcius = new double[] {rollersTemperature.getValueAsDouble()};

    inputs.intakePositionRad =
        Rotation2d.fromRadians(
            Units.rotationsToRadians(intakePosition.getValueAsDouble()) / INTAKE_GEAR_RATIO);
    inputs.intakeVelocityRadPerSec =
        Units.rotationsToRadians(intakeVelocity.getValueAsDouble()) / INTAKE_GEAR_RATIO;
    inputs.intakeAppliedVolts = intakeAppliedVolts.getValueAsDouble();
    inputs.intakeCurrentAmps = new double[] {intakeCurrent.getValueAsDouble()};
    inputs.intakeTempCelcius = new double[] {intakeTemperature.getValueAsDouble()};
  }

  @Override
  public void setRollersVoltage(double volts) {
    rollersTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setIntakeVoltage(double volts) {
    intakeTalon.setControl(new VoltageOut(volts));
  }
}
