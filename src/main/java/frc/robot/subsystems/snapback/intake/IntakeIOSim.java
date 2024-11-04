package frc.robot.subsystems.snapback.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import frc.robot.constants.Constants;

public class IntakeIOSim implements IntakeIO {
  private DCMotorSim intakeMotorSim =
      new DCMotorSim(IntakeConstants.MOTOR_CONFIG, IntakeConstants.INTAKE_GEAR_REDUCTION, 0.004);
  private DCMotorSim serializerMotorSim =
      new DCMotorSim(
          IntakeConstants.MOTOR_CONFIG, IntakeConstants.SERIALIZER_GEAR_REDUCTION, 0.004);
  private DCMotorSim kickerMotorSim =
      new DCMotorSim(IntakeConstants.MOTOR_CONFIG, IntakeConstants.KICKER_GEAR_REDUCTION, 0.004);

  private DoubleSolenoidSim SOLENOID_SIM =
      new DoubleSolenoidSim(PneumaticsModuleType.CTREPCM, 5, 6);

  private double intakeMotorAppliedVolts = 0.0;
  private double serializerMotorAppliedVolts = 0.0;
  private double kickerMotorAppliedVolts = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeMotorSim.update(Constants.LOOP_PERIOD_SECONDS);
    serializerMotorSim.update(Constants.LOOP_PERIOD_SECONDS);
    kickerMotorSim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.intakePosition = Rotation2d.fromRadians(intakeMotorSim.getAngularPositionRad());
    inputs.intakeVelocityRadiansPerSecond = intakeMotorSim.getAngularVelocityRadPerSec();
    inputs.intakeAppliedVolts = intakeMotorAppliedVolts;
    inputs.intakeCurrentAmps = intakeMotorSim.getCurrentDrawAmps();

    inputs.serializerPosition = Rotation2d.fromRadians(serializerMotorSim.getAngularPositionRad());
    inputs.serializerVelocityRadiansPerSecond = serializerMotorSim.getAngularVelocityRadPerSec();
    inputs.serializerAppliedVolts = serializerMotorAppliedVolts;
    inputs.serializerCurrentAmps = serializerMotorSim.getCurrentDrawAmps();

    inputs.kickerPosition = Rotation2d.fromRadians(kickerMotorSim.getAngularPositionRad());
    inputs.kickerVelocityRadiansPerSecond = kickerMotorSim.getAngularVelocityRadPerSec();
    inputs.kickerAppliedVolts = kickerMotorAppliedVolts;
    inputs.kickerCurrentAmps = kickerMotorSim.getCurrentDrawAmps();

    inputs.sensorValue = false;
    inputs.pneumaticValue = SOLENOID_SIM.get();
  }

  @Override
  public void setIntakeVoltage(double volts) {
    intakeMotorAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    intakeMotorSim.setInputVoltage(intakeMotorAppliedVolts);
  }

  @Override
  public void setSerializerVoltage(double volts) {
    serializerMotorAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    serializerMotorSim.setInputVoltage(serializerMotorAppliedVolts);
  }

  @Override
  public void setKickerVoltage(double volts) {
    kickerMotorAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    kickerMotorSim.setInputVoltage(kickerMotorAppliedVolts);
  }

  @Override
  public void setActuatorValue(Value value) {
    SOLENOID_SIM.set(value);
  }
}
