package frc.robot.subsystems.whiplash.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
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

public class WhiplashShooterIOTalonFX implements WhiplashShooterIO {
  private final TalonFX topMotor;
  private final TalonFX bottomMotor;

  private final StatusSignal<Angle> topPositionRotations;
  private final StatusSignal<AngularVelocity> topVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> topAppliedVolts;
  private final StatusSignal<Current> topCurrentAmps;
  private final StatusSignal<Temperature> topTemperatureCelsius;
  private double topGoalRadiansPerSecond;
  private final StatusSignal<Double> topVelocitySetpointRotationsPerSecond;
  private final StatusSignal<Double> topVelocityErrorRotationsPerSecond;

  private final StatusSignal<Angle> bottomPositionRotations;
  private final StatusSignal<AngularVelocity> bottomVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> bottomAppliedVolts;
  private final StatusSignal<Current> bottomCurrentAmps;
  private final StatusSignal<Temperature> bottomTemperatureCelsius;
  private double bottomGoalRadiansPerSecond;
  private final StatusSignal<Double> bottomVelocitySetpointRotationsPerSecond;
  private final StatusSignal<Double> bottomVelocityErrorRotationsPerSecond;

  private final TalonFXConfiguration topConfig;
  private final TalonFXConfiguration bottomConfig;

  private final NeutralOut neutralControlRequest;
  private final VoltageOut voltageControlRequest;
  private final VelocityVoltage topVelocityControlRequest;
  private final VelocityVoltage bottomVelocityControlRequest;

  public WhiplashShooterIOTalonFX() {
    topMotor = new TalonFX(WhiplashShooterConstants.TOP_CAN_ID);
    bottomMotor = new TalonFX(WhiplashShooterConstants.BOTTOM_CAN_ID);

    topConfig = new TalonFXConfiguration();
    bottomConfig = new TalonFXConfiguration();

    topConfig.CurrentLimits.SupplyCurrentLimit = WhiplashShooterConstants.CURRENT_LIMIT;
    topConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    topConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    topConfig.Slot0.kP = WhiplashShooterConstants.GAINS.kp().get();
    topConfig.Slot0.kD = WhiplashShooterConstants.GAINS.kd().get();
    topConfig.Slot0.kS = WhiplashShooterConstants.GAINS.ks().get();
    topConfig.Slot0.kV = WhiplashShooterConstants.GAINS.kv().get();
    topConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    bottomConfig.CurrentLimits.SupplyCurrentLimit = WhiplashShooterConstants.CURRENT_LIMIT;
    bottomConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    bottomConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    bottomConfig.Slot0.kP = WhiplashShooterConstants.GAINS.kp().get();
    bottomConfig.Slot0.kD = WhiplashShooterConstants.GAINS.kd().get();
    bottomConfig.Slot0.kS = WhiplashShooterConstants.GAINS.ks().get();
    bottomConfig.Slot0.kV = WhiplashShooterConstants.GAINS.kv().get();

    topMotor.getConfigurator().apply(topConfig);
    bottomMotor.getConfigurator().apply(bottomConfig);

    topPositionRotations = topMotor.getPosition();
    topVelocityRotationsPerSecond = topMotor.getVelocity();
    topAppliedVolts = topMotor.getMotorVoltage();
    topCurrentAmps = topMotor.getSupplyCurrent();
    topTemperatureCelsius = topMotor.getDeviceTemp();
    topGoalRadiansPerSecond = 0.0;
    topVelocitySetpointRotationsPerSecond = topMotor.getClosedLoopReference();
    topVelocityErrorRotationsPerSecond = topMotor.getClosedLoopError();

    bottomPositionRotations = bottomMotor.getPosition();
    bottomVelocityRotationsPerSecond = bottomMotor.getVelocity();
    bottomAppliedVolts = bottomMotor.getMotorVoltage();
    bottomCurrentAmps = bottomMotor.getSupplyCurrent();
    bottomTemperatureCelsius = bottomMotor.getDeviceTemp();
    bottomGoalRadiansPerSecond = 0.0;
    bottomVelocitySetpointRotationsPerSecond = bottomMotor.getClosedLoopReference();
    bottomVelocityErrorRotationsPerSecond = bottomMotor.getClosedLoopError();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        topPositionRotations,
        topVelocityRotationsPerSecond,
        topAppliedVolts,
        topCurrentAmps,
        topTemperatureCelsius,
        topVelocitySetpointRotationsPerSecond,
        topVelocityErrorRotationsPerSecond,
        bottomPositionRotations,
        bottomVelocityRotationsPerSecond,
        bottomAppliedVolts,
        bottomCurrentAmps,
        bottomTemperatureCelsius,
        bottomVelocitySetpointRotationsPerSecond,
        bottomVelocityErrorRotationsPerSecond);

    topMotor.optimizeBusUtilization(50.0, 1.0);
    bottomMotor.optimizeBusUtilization(50.0, 1.0);

    neutralControlRequest = new NeutralOut();
    voltageControlRequest = new VoltageOut(0.0);
    topVelocityControlRequest = new VelocityVoltage(0);
    bottomVelocityControlRequest = new VelocityVoltage(0);
  }

  @Override
  public void updateInputs(WhiplashShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        topPositionRotations,
        topVelocityRotationsPerSecond,
        topAppliedVolts,
        topCurrentAmps,
        topTemperatureCelsius,
        topVelocitySetpointRotationsPerSecond,
        topVelocityErrorRotationsPerSecond,
        bottomPositionRotations,
        bottomVelocityRotationsPerSecond,
        bottomAppliedVolts,
        bottomCurrentAmps,
        bottomTemperatureCelsius,
        bottomVelocitySetpointRotationsPerSecond,
        bottomVelocityErrorRotationsPerSecond);
    topVelocitySetpointRotationsPerSecond.refresh();
    topVelocityErrorRotationsPerSecond.refresh();
    bottomVelocitySetpointRotationsPerSecond.refresh();
    bottomVelocityErrorRotationsPerSecond.refresh();

    inputs.topPosition = Rotation2d.fromRotations(topPositionRotations.getValueAsDouble());
    inputs.topVelocityRadiansPerSecond =
        Units.rotationsToRadians(topVelocityRotationsPerSecond.getValueAsDouble());
    inputs.topAppliedVolts = topAppliedVolts.getValueAsDouble();
    inputs.topCurrentAmps = topCurrentAmps.getValueAsDouble();
    inputs.topTemperatureCelsius = topTemperatureCelsius.getValueAsDouble();
    inputs.topVelocityGoalRadiansPerSecond = topGoalRadiansPerSecond;
    inputs.topVelocitySetpointRadiansPerSecond =
        Units.rotationsToRadians(topVelocitySetpointRotationsPerSecond.getValueAsDouble());
    inputs.topVelocityErrorRadiansPerSecond =
        Units.rotationsToRadians(topVelocityErrorRotationsPerSecond.getValueAsDouble());

    inputs.bottomPosition = Rotation2d.fromRotations(bottomPositionRotations.getValueAsDouble());
    inputs.bottomVelocityRadiansPerSecond =
        Units.rotationsToRadians(bottomVelocityRotationsPerSecond.getValueAsDouble());
    inputs.bottomAppliedVolts = bottomAppliedVolts.getValueAsDouble();
    inputs.bottomCurrentAmps = bottomCurrentAmps.getValueAsDouble();
    inputs.bottomTemperatureCelsius = bottomTemperatureCelsius.getValueAsDouble();
    inputs.bottomVelocityGoalRadiansPerSecond = bottomGoalRadiansPerSecond;
    inputs.bottomVelocitySetpointRadiansPerSecond =
        Units.rotationsToRadians(bottomVelocitySetpointRotationsPerSecond.getValueAsDouble());
    inputs.bottomVelocityErrorRadiansPerSecond =
        Units.rotationsToRadians(bottomVelocityErrorRotationsPerSecond.getValueAsDouble());
  }

  @Override
  public void setVoltage(double volts) {
    topMotor.setControl(voltageControlRequest.withOutput(volts).withEnableFOC(true));
    bottomMotor.setControl(voltageControlRequest.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setTopVelocity(double velocityRadiansPerSecond) {
    topGoalRadiansPerSecond = velocityRadiansPerSecond;
    topMotor.setControl(
        topVelocityControlRequest
            .withVelocity(Units.radiansToRotations(velocityRadiansPerSecond))
            .withEnableFOC(true));
  }

  @Override
  public void setBottomVelocity(double velocityRadiansPerSecond) {
    bottomGoalRadiansPerSecond = velocityRadiansPerSecond;
    bottomMotor.setControl(
        bottomVelocityControlRequest
            .withVelocity(Units.radiansToRotations(velocityRadiansPerSecond))
            .withEnableFOC(true));
  }

  @Override
  public void setPID(double kp, double ki, double kd) {
    topConfig.Slot0.kP = kp;
    topConfig.Slot0.kI = ki;
    topConfig.Slot0.kD = kd;
    topMotor.getConfigurator().apply(topConfig, 0.01);
  }

  @Override
  public void setFeedforward(double ks, double kv, double ka) {
    topConfig.Slot0.kS = ks;
    topConfig.Slot0.kV = kv;
    topConfig.Slot0.kA = ka;
    topMotor.getConfigurator().apply(topConfig, 0.01);
  }

  @Override
  public void setProfile(
      double maxAccelerationRadiansPerSecondSquared, double goalToleranceRadiansPerSecond) {
    topConfig.MotionMagic.MotionMagicAcceleration =
        Units.radiansToRotations(maxAccelerationRadiansPerSecondSquared);
    topMotor.getConfigurator().apply(topConfig);
  }

  @Override
  public boolean atGoal() {
    return Math.abs(
                Units.radiansToRotations(topGoalRadiansPerSecond)
                    - topVelocityRotationsPerSecond.getValueAsDouble())
            <= Units.radiansToRotations(
                WhiplashShooterConstants.CONSTRAINTS.goalToleranceRadiansPerSecond().get())
        && Math.abs(
                Units.radiansToRotations(bottomGoalRadiansPerSecond)
                    - bottomVelocityRotationsPerSecond.getValueAsDouble())
            <= Units.radiansToRotations(
                WhiplashShooterConstants.CONSTRAINTS.goalToleranceRadiansPerSecond().get());
  }

  @Override
  public void stop() {
    topMotor.setControl(neutralControlRequest);
    bottomMotor.setControl(neutralControlRequest);
  }
}
