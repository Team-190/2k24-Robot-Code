package frc.robot.subsystems.snapback.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import frc.robot.Constants;

public class SnapbackIntakeIOSim implements SnapbackIntakeIO {

  private DCMotorSim intakeMotorSim;
  private DCMotorSim serializerMotorSim;
  private DCMotorSim kickerMotorSim;

  private DoubleSolenoidSim solenoidSim;

  public SnapbackIntakeIOSim() {
    intakeMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                SnapbackIntakeConstants.INTAKE_GEARBOX,
                0.004,
                SnapbackIntakeConstants.INTAKE_GEAR_REDUCTION),
            SnapbackIntakeConstants.INTAKE_GEARBOX);

    serializerMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                SnapbackIntakeConstants.SERIALIZER_GEARBOX,
                0.004,
                SnapbackIntakeConstants.SERIALIZER_GEAR_REDUCTION),
            SnapbackIntakeConstants.SERIALIZER_GEARBOX);

    kickerMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                SnapbackIntakeConstants.KICKER_GEARBOX,
                0.004,
                SnapbackIntakeConstants.KICKER_GEAR_REDUCTION),
            SnapbackIntakeConstants.KICKER_GEARBOX);

    solenoidSim = new DoubleSolenoidSim(PneumaticsModuleType.CTREPCM, 5, 6);
  }

  @Override
  public void updateInputs(SnapbackIntakeIOInputs inputs) {
    intakeMotorSim.update(Constants.LOOP_PERIOD_SECONDS);
    serializerMotorSim.update(Constants.LOOP_PERIOD_SECONDS);
    kickerMotorSim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.intakePosition = Rotation2d.fromRadians(intakeMotorSim.getAngularPositionRad());
    inputs.intakeVelocityRadiansPerSecond = intakeMotorSim.getAngularVelocityRadPerSec();
    inputs.intakeAppliedVolts = intakeMotorSim.getInputVoltage();
    inputs.intakeCurrentAmps = intakeMotorSim.getCurrentDrawAmps();

    inputs.serializerPosition = Rotation2d.fromRadians(serializerMotorSim.getAngularPositionRad());
    inputs.serializerVelocityRadiansPerSecond = serializerMotorSim.getAngularVelocityRadPerSec();
    inputs.serializerAppliedVolts = serializerMotorSim.getInputVoltage();
    inputs.serializerCurrentAmps = serializerMotorSim.getCurrentDrawAmps();

    inputs.kickerPosition = Rotation2d.fromRadians(kickerMotorSim.getAngularPositionRad());
    inputs.kickerVelocityRadiansPerSecond = kickerMotorSim.getAngularVelocityRadPerSec();
    inputs.kickerAppliedVolts = kickerMotorSim.getInputVoltage();
    inputs.kickerCurrentAmps = kickerMotorSim.getCurrentDrawAmps();

    inputs.sensorValue = false;
    inputs.pneumaticValue = solenoidSim.get();
  }

  @Override
  public void setIntakeVoltage(double volts) {
    intakeMotorSim.setInputVoltage(volts);
  }

  @Override
  public void setSerializerVoltage(double volts) {
    serializerMotorSim.setInputVoltage(volts);
  }

  @Override
  public void setKickerVoltage(double volts) {
    kickerMotorSim.setInputVoltage(volts);
  }

  @Override
  public void setActuatorValue(Value value) {
    solenoidSim.set(value);
  }
}
