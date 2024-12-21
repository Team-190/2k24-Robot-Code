package frc.robot.subsystems.snapback.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.snapback.hood.SnapbackHoodConstants.SnapbackHoodGoal;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SnapbackHood extends SubsystemBase {
  private final SnapbackHoodIO io;
  private final SnapbackHoodIOInputsAutoLogged inputs;

  private SysIdRoutine characterizationRoutine;

  private boolean isClosedLoop;
  private SnapbackHoodGoal goal;

  public SnapbackHood(SnapbackHoodIO io) {
    inputs = new SnapbackHoodIOInputsAutoLogged();
    this.io = io;

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Seconds),
                Volts.of(3.5),
                Seconds.of(10),
                (state) -> Logger.recordOutput("Arm/sysIDState", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.setVoltage(volts.in(Volts)), null, this));

    isClosedLoop = false;
    goal = SnapbackHoodGoal.STOW;
  }

  public void periodic() {
    switch (Constants.ROBOT) {
      case SNAPBACK:
      case SNAPBACK_SIM:
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);

        if (isClosedLoop) {
          io.setPosition(goal.getAngle());
        }
        break;
      default:
        break;
    }
  }

  public Command setGoal(SnapbackHoodGoal goal) {
    isClosedLoop = true;
    return Commands.run(() -> this.goal = goal);
  }

  @AutoLogOutput(key = "Hood/At Goal")
  public boolean atGoal() {
    return io.atGoal();
  }

  public Command increaseAngle() {
    return Commands.runOnce(
        () ->
            RobotState.setSpeakerAngleCompensation(
                RobotState.getSpeakerAngleCompensation() + Units.degreesToRadians(0.25)));
  }

  public Command decreaseAngle() {
    return Commands.runOnce(
        () ->
            RobotState.setSpeakerAngleCompensation(
                RobotState.getSpeakerAngleCompensation() - Units.degreesToRadians(0.25)));
  }

  public void setPID(double kp, double kd) {
    io.setPID(kp, 0.0, kd);
  }

  public void setFeedforward(double ks, double kv, double ka) {
    io.setFeedforward(ks, kv, ka);
  }

  public void setProfile(
      double maxVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared,
      double goalToleranceRadians) {
    io.setProfile(
        maxVelocityRadiansPerSecond, maxAccelerationRadiansPerSecondSquared, goalToleranceRadians);
  }

  public Command runSysId() {
    isClosedLoop = false;
    return Commands.sequence(
        characterizationRoutine.quasistatic(Direction.kForward),
        Commands.waitSeconds(3),
        characterizationRoutine.quasistatic(Direction.kReverse),
        Commands.waitSeconds(3),
        characterizationRoutine.dynamic(Direction.kForward),
        Commands.waitSeconds(3),
        characterizationRoutine.dynamic(Direction.kReverse));
  }
}
