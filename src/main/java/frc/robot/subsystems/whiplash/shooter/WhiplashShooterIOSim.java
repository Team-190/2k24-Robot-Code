package frc.robot.subsystems.whiplash.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.util.LinearProfile;

public class WhiplashShooterIOSim implements WhiplashShooterIO {

  private DCMotorSim topMotorSim;
  private DCMotorSim bottomMotorSim;
  private SimpleMotorFeedforward feedForward;
  private final PIDController feedback;
  private final LinearProfile profile;
  private double topAppliedVolts;
  private double bottomAppliedVolts;

  public WhiplashShooterIOSim() {
    topMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                WhiplashShooterConstants.TOP_MOTOR_CONFIG, 0.004, 1.0),
            WhiplashShooterConstants.TOP_MOTOR_CONFIG);
    bottomMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                WhiplashShooterConstants.BOTTOM_MOTOR_CONFIG, 0.004, 1.0),
            WhiplashShooterConstants.BOTTOM_MOTOR_CONFIG);

    feedForward =
        new SimpleMotorFeedforward(
            WhiplashShooterConstants.KS.get(), WhiplashShooterConstants.KV.get());
    feedback =
        new PIDController(
            WhiplashShooterConstants.KP.get(), 0.0, WhiplashShooterConstants.KD.get());
    profile =
        new LinearProfile(
            WhiplashShooterConstants.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED.get(),
            Constants.LOOP_PERIOD_SECONDS);
    feedback.setTolerance(WhiplashShooterConstants.SPEED_TOLERANCE_RADIANS_PER_SECOND);
    topAppliedVolts = 0.0;
    bottomAppliedVolts = 0.0;
  }

  @Override
  public void updateInputs(WhiplashShooterIOInputs inputs) {

    topMotorSim.update(Constants.LOOP_PERIOD_SECONDS);
    bottomMotorSim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.topPosition = Rotation2d.fromRadians(topMotorSim.getAngularPositionRad());
    inputs.bottomPosition = Rotation2d.fromRadians(bottomMotorSim.getAngularPositionRad());
    inputs.topVelocityRadPerSec = topMotorSim.getAngularVelocityRadPerSec();
    inputs.bottomVelocityRadPerSec = bottomMotorSim.getAngularVelocityRadPerSec();
    inputs.topAppliedVolts = topAppliedVolts;
    inputs.bottomAppliedVolts = bottomAppliedVolts;

    inputs.topVelocityGoalRadiansPerSec = profile.getGoal();
    inputs.bottomVelocityGoalRadiansPerSec = profile.getGoal();

    inputs.topVelocitySetpointRadiansPerSec = feedback.getSetpoint();
    inputs.bottomVelocitySetpointRadiansPerSec = feedback.getSetpoint();

    inputs.topVelocityErrorRadiansPerSec = feedback.getError();
    inputs.bottomVelocityErrorRadiansPerSec = feedback.getError();
  }

  /**
   * Sets the voltage of the Top Motor to a number between negative 12 and 12. Specifically, this
   * number is calculated using the feedForward gains.
   *
   * @param setPointVelocityRadiansPerSecond the target for the velocity, in radians per second, to
   *     reach.
   */
  @Override
  public void setTopVelocitySetPoint(double setPointVelocityRadiansPerSecond) {
    profile.setGoal(setPointVelocityRadiansPerSecond, topMotorSim.getAngularVelocityRadPerSec());
    topAppliedVolts =
        MathUtil.clamp(
            feedback.calculate(
                    topMotorSim.getAngularVelocityRadPerSec(), profile.calculateSetpoint())
                + feedForward.calculate(feedback.getSetpoint()),
            -12.0,
            12.0);
    topMotorSim.setInputVoltage(topAppliedVolts);
  }

  /**
   * Sets the voltage of the Bottom Motor to a number between negative 12 and 12. Specifically, this
   * number is calculated using the feedForward gains.
   *
   * @param setPointVelocityRadiansPerSecond the target for the velocity, in radians per second, to
   *     reach.
   */
  @Override
  public void setBottomVelocitySetPoint(double setPointVelocityRadiansPerSecond) {
    profile.setGoal(setPointVelocityRadiansPerSecond, bottomMotorSim.getAngularVelocityRadPerSec());
    bottomAppliedVolts =
        MathUtil.clamp(
            feedback.calculate(
                    bottomMotorSim.getAngularVelocityRadPerSec(), profile.calculateSetpoint())
                + feedForward.calculate(feedback.getSetpoint()),
            -12.0,
            12.0);
    bottomMotorSim.setInputVoltage(bottomAppliedVolts);
  }

  /**
   * Takes in a value volts, and sets the voltage of both the top and bottom motors to this value.
   * The value volts is between negative 12 and 12.
   *
   * @param volts the voltage that will be inputted as the input voltage for the top and bottom
   *     motors.
   */
  @Override
  public void setVoltage(double volts) {

    topAppliedVolts = MathUtil.clamp(volts, -12, 12);
    bottomAppliedVolts = MathUtil.clamp(volts, -12, 12);
    topMotorSim.setInputVoltage(topAppliedVolts);
    bottomMotorSim.setInputVoltage(topAppliedVolts);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    feedback.setPID(kP, kI, kD);
  }

  @Override
  public void setFeedForward(double kS, double kV, double kA) {
    feedForward = new SimpleMotorFeedforward(kS, kV, kA);
  }

  @Override
  public void setProfile(double maxAcceleration) {
    profile.setMaxAcceleration(maxAcceleration);
  }

  @Override
  public boolean atSetPoint() {
    return (Math.abs(profile.getGoal() - feedback.getSetpoint())
            <= WhiplashShooterConstants.SPEED_TOLERANCE_RADIANS_PER_SECOND)
        && feedback.atSetpoint();
  }

  /** Stops the robot by setting the voltage of both motors to 0. */
  @Override
  public void stop() {

    topAppliedVolts = 0.0;
    bottomAppliedVolts = 0.0;
    topMotorSim.setInputVoltage(0.0);
    bottomMotorSim.setInputVoltage(0.0);
  }
}
