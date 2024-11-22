package frc.robot.subsystems.snapback.climber;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.Constants;

public class ClimberIOSim implements ClimberIO {
  private final DCMotorSim leftClimberMotorSim;
  private final DCMotorSim rightClimberMotorSim;

  private double leftClimberMotorAppliedVolts;
  private double rightClimberMotorAppliedVolts;

  private ProfiledPIDController leftClimberPIDController;
  private ProfiledPIDController rightClimberPIDController;
  private SimpleMotorFeedforward leftClimberFeedforward;
  private SimpleMotorFeedforward rightClimberFeedforward;

  public ClimberIOSim() {
    leftClimberMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ClimberConstants.CLIMBER_GEARBOX, 0.004, ClimberConstants.CLIMBER_GEAR_REDUCTION),
            ClimberConstants.CLIMBER_GEARBOX,
            0.004);
    rightClimberMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ClimberConstants.CLIMBER_GEARBOX, 0.004, ClimberConstants.CLIMBER_GEAR_REDUCTION),
            ClimberConstants.CLIMBER_GEARBOX,
            0.004);

    leftClimberMotorAppliedVolts = 0.0;
    rightClimberMotorAppliedVolts = 0.0;

    leftClimberPIDController =
        new ProfiledPIDController(
            ClimberConstants.GAINS.kp().get(),
            ClimberConstants.GAINS.ki().get(),
            ClimberConstants.GAINS.kd().get(),
            new TrapezoidProfile.Constraints(
                ClimberConstants.GAINS.ka(), Double.POSITIVE_INFINITY));
    rightClimberPIDController =
        new ProfiledPIDController(
            ClimberConstants.GAINS.kp().get(),
            ClimberConstants.GAINS.ki().get(),
            ClimberConstants.GAINS.kd().get(),
            new TrapezoidProfile.Constraints(
                ClimberConstants.GAINS.ka(), Double.POSITIVE_INFINITY));
    leftClimberPIDController.setTolerance(ClimberConstants.CLIMBER_TOLERANCE_METERS.get());
    rightClimberPIDController.setTolerance(ClimberConstants.CLIMBER_TOLERANCE_METERS.get());

    leftClimberFeedforward =
        new SimpleMotorFeedforward(
            ClimberConstants.GAINS.ks(), ClimberConstants.GAINS.kv(), ClimberConstants.GAINS.ka());
    rightClimberFeedforward =
        new SimpleMotorFeedforward(
            ClimberConstants.GAINS.ks(), ClimberConstants.GAINS.kv(), ClimberConstants.GAINS.ka());
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {

    leftClimberMotorSim.update(Constants.LOOP_PERIOD_SECONDS);
    rightClimberMotorSim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.leftPosition =
        leftClimberMotorSim.getAngularPositionRad() * ClimberConstants.CLIMBER_PULLY_RADIUS_METERS;
    inputs.leftVelocityRadiansPerSecond = leftClimberMotorSim.getAngularVelocityRadPerSec();
    inputs.leftAppliedVolts = leftClimberMotorSim.getInputVoltage();
    inputs.leftCurrentAmps = leftClimberMotorSim.getCurrentDrawAmps();
    inputs.leftPositionGoalMeters =
        leftClimberPIDController.getSetpoint().position
            * 2
            * Math.PI
            * ClimberConstants.CLIMBER_PULLY_RADIUS_METERS;
    inputs.leftPositionErrorMeters = leftClimberPIDController.getPositionError();

    inputs.rightPosition =
        rightClimberMotorSim.getAngularPositionRad() * ClimberConstants.CLIMBER_PULLY_RADIUS_METERS;
    inputs.rightVelocityRadiansPerSecond = rightClimberMotorSim.getAngularVelocityRadPerSec();
    inputs.rightAppliedVolts = rightClimberMotorSim.getInputVoltage();
    inputs.rightCurrentAmps = rightClimberMotorSim.getCurrentDrawAmps();
    inputs.rightPositionGoalMeters =
        rightClimberPIDController.getSetpoint().position
            * 2
            * Math.PI
            * ClimberConstants.CLIMBER_PULLY_RADIUS_METERS;
    inputs.rightPositionErrorMeters =
        rightClimberPIDController.getPositionError()
            * 2
            * Math.PI
            * ClimberConstants.CLIMBER_PULLY_RADIUS_METERS;
  }

  @Override
  public void setLeftVoltage(double volts) {
    leftClimberMotorAppliedVolts = volts;
    leftClimberMotorSim.setInputVoltage(volts);
  }

  @Override
  public void setRightVoltage(double volts) {
    rightClimberMotorAppliedVolts = volts;
    rightClimberMotorSim.setInputVoltage(volts);
  }

  @Override
  public void setLeftPositionGoal(double positionMeters) {
    double positionsRotations =
        positionMeters / (2 * Math.PI * ClimberConstants.CLIMBER_PULLY_RADIUS_METERS);
    leftClimberMotorSim.setInputVoltage(
        leftClimberPIDController.calculate(positionsRotations)
            + leftClimberFeedforward.calculate(leftClimberPIDController.getSetpoint().position));
  }

  @Override
  public void setRightPositionGoal(double positionMeters) {
    double positionsRotations =
        positionMeters / (2 * Math.PI * ClimberConstants.CLIMBER_PULLY_RADIUS_METERS);
    rightClimberMotorSim.setInputVoltage(
        rightClimberPIDController.calculate(positionsRotations)
            + rightClimberFeedforward.calculate(rightClimberPIDController.getSetpoint().position));
  }

  @Override
  public void setLeftPosition(double positionMeters) {
    double positionsRotations =
        positionMeters / (2 * Math.PI * ClimberConstants.CLIMBER_PULLY_RADIUS_METERS);
    leftClimberMotorSim.setAngle(Units.rotationsToRadians(positionsRotations));
  }

  @Override
  public void setRightPosition(double positionMeters) {
    double positionsRotations =
        positionMeters / (2 * Math.PI * ClimberConstants.CLIMBER_PULLY_RADIUS_METERS);
    rightClimberMotorSim.setAngle(Units.rotationsToRadians(positionsRotations));
  }

  @Override
  public boolean atGoal() {
    return leftClimberPIDController.atGoal() && rightClimberPIDController.atGoal();
  }
}
