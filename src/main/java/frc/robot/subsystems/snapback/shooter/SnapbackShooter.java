package frc.robot.subsystems.snapback.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.subsystems.snapback.shooter.SnapbackShooterConstants.SnapbackShooterGoal;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SnapbackShooter extends SubsystemBase {

  private final SnapbackShooterIO io;
  private final SnapbackShooterIOInputsAutoLogged inputs;
  private SnapbackShooterGoal goal;
  private boolean isClosedLoop;

  private final SysIdRoutine sysIdRoutineLeft;
  private final SysIdRoutine sysIdRoutineRight;

  public SnapbackShooter(SnapbackShooterIO io) {
    this.io = io;
    inputs = new SnapbackShooterIOInputsAutoLogged();

    goal = SnapbackShooterGoal.IDLE;
    isClosedLoop = false;

    sysIdRoutineLeft =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(3.5),
                Seconds.of(10.0),
                (state) -> Logger.recordOutput("Shooter/sysID State Left", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.setLeftVoltage(volts.in(Volts)), null, this));

    sysIdRoutineRight =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(3.5),
                Seconds.of(10.0),
                (state) -> Logger.recordOutput("Shooter/sysID State Right", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.setRightVoltage(volts.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    switch (Constants.ROBOT) {
      case SNAPBACK:
      case SNAPBACK_SIM:
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        if (isClosedLoop) {
          io.setLeftVelocity(goal.getLeftGoal());
          io.setRightVelocity(goal.getRightGoal());
        }
        break;
      default:
        break;
    }
  }

  public Command setGoal(SnapbackShooterGoal goal) {
    return runOnce(() -> this.goal = goal);
  }

  @AutoLogOutput(key = "Shooter/At Goal")
  public boolean atGoal() {
    return io.atGoal();
  }

  public void setPID(double kp, double kd) {
    io.setPID(kp, 0.0, kd);
  }

  public void setFeedforward(double ks, double kv, double ka) {
    io.setFeedforward(ks, kv, ka);
  }

  public void setProfile(
      double maxAccelerationRadiansPerSecondSquared, double goalToleranceRadians) {
    io.setProfile(maxAccelerationRadiansPerSecondSquared, goalToleranceRadians);
  }

  public Command runSysId() {
    isClosedLoop = false;
    return Commands.sequence(
        sysIdRoutineLeft
            .quasistatic(Direction.kForward)
            .alongWith(sysIdRoutineLeft.quasistatic(Direction.kForward)),
        Commands.waitSeconds(4),
        sysIdRoutineRight
            .quasistatic(Direction.kReverse)
            .alongWith(sysIdRoutineRight.quasistatic(Direction.kReverse)),
        Commands.waitSeconds(4),
        sysIdRoutineLeft
            .dynamic(Direction.kForward)
            .alongWith(sysIdRoutineLeft.dynamic(Direction.kForward)),
        Commands.waitSeconds(4),
        sysIdRoutineRight
            .dynamic(Direction.kReverse)
            .alongWith(sysIdRoutineRight.dynamic(Direction.kReverse)));
  }
}
