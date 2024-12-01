package frc.robot.subsystems.snapback.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import frc.robot.constants.Constants;

public class IntakeIOSim implements IntakeIO {

  private DCMotorSim intakeMotorSim;
  private DCMotorSim serializerMotorSim;
  private DCMotorSim kickerMotorSim;

  private DoubleSolenoidSim solenoidSim;

  public IntakeIOSim() {
    intakeMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                IntakeConstants.INTAKE_GEARBOX, 0.004, IntakeConstants.INTAKE_GEAR_REDUCTION),
            IntakeConstants.INTAKE_GEARBOX,
            0.004);

    serializerMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                IntakeConstants.SERIALIZER_GEARBOX,
                0.004,
                IntakeConstants.SERIALIZER_GEAR_REDUCTION),
            IntakeConstants.SERIALIZER_GEARBOX,
            0.004);

    kickerMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                IntakeConstants.KICKER_GEARBOX, 0.004, IntakeConstants.KICKER_GEAR_REDUCTION),
            IntakeConstants.KICKER_GEARBOX,
            0.004);

    solenoidSim = new DoubleSolenoidSim(PneumaticsModuleType.CTREPCM, 5, 6);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
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
