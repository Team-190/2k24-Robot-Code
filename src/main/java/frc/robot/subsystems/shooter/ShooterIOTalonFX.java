package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX leaderTalon;
  private final TalonFX followerTalon;

  private final StatusSignal<Double> leaderPosition;
  private final StatusSignal<Double> leaderVelocity;
  private final StatusSignal<Double> leaderAppliedVolts;
  private final StatusSignal<Double> leaderCurrent;
  private final StatusSignal<Double> followerCurrent;
  private final StatusSignal<Double> leaderTemperature;
  private final StatusSignal<Double> followerTemperature;

  private final double GEAR_RATIO = 47.0 / 17.0;

  public ShooterIOTalonFX() {
    switch (Constants.ROBOT) {
      case ROBOT_2K24_C:
        leaderTalon = new TalonFX(10);
        followerTalon = new TalonFX(11);
        break;
      case ROBOT_2K24_P:
        leaderTalon = new TalonFX(10);
        followerTalon = new TalonFX(11);
        break;
      case ROBOT_2K24_TEST:
        leaderTalon = new TalonFX(10);
        followerTalon = new TalonFX(11);
        break;
      default:
        throw new RuntimeException("Invalid robot");
    }

    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    leaderTalon.getConfigurator().apply(config);
    followerTalon.getConfigurator().apply(config);
    followerTalon.setControl(new Follower(leaderTalon.getDeviceID(), true));

    leaderPosition = leaderTalon.getPosition();
    leaderVelocity = leaderTalon.getVelocity();
    leaderAppliedVolts = leaderTalon.getMotorVoltage();
    leaderCurrent = leaderTalon.getStatorCurrent();
    followerCurrent = followerTalon.getStatorCurrent();
    leaderTemperature = leaderTalon.getDeviceTemp();
    followerTemperature = followerTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, leaderVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leaderPosition,
        leaderAppliedVolts,
        leaderCurrent,
        followerCurrent,
        leaderTemperature,
        followerTemperature);
    leaderTalon.optimizeBusUtilization();
    followerTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leaderPosition,
        leaderVelocity,
        leaderAppliedVolts,
        leaderCurrent,
        followerCurrent,
        leaderTemperature,
        followerTemperature);

    inputs.positionRad = Units.rotationsToRadians(leaderPosition.getValueAsDouble()) * GEAR_RATIO;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(leaderVelocity.getValueAsDouble()) * GEAR_RATIO;
    inputs.appliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {leaderCurrent.getValueAsDouble(), followerCurrent.getValueAsDouble()};
    inputs.tempCelcius =
        new double[] {leaderTemperature.getValueAsDouble(), followerTemperature.getValueAsDouble()};
  }

  @Override
  public void setVoltage(double volts) {
    leaderTalon.setControl(new VoltageOut(volts));
  }
}
