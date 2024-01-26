package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private static final LoggedTunableNumber KS = new LoggedTunableNumber("Shooter/Ks");
  private static final LoggedTunableNumber KV = new LoggedTunableNumber("Shooter/Kv");
  private static final LoggedTunableNumber KP = new LoggedTunableNumber("Shooter/Kp");
  private static final LoggedTunableNumber KD = new LoggedTunableNumber("Shooter/Kd");

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private SimpleMotorFeedforward feedforward;
  private final PIDController feedback;
  private boolean isOpenLoop = true;
  private double openLoopVoltage = 0.0;
  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              null,
              null,
              (state) -> Logger.recordOutput("Shooter/sysIDState", state.toString())),
          new SysIdRoutine.Mechanism((volts) -> setVoltage(volts.in(Volts)), null, this));

  static {
    switch (Constants.ROBOT) {
      case ROBOT_2K24_C:
      case ROBOT_2K24_P:
        KS.initDefault(0.0);
        KV.initDefault(0.0);
        KP.initDefault(0.0);
        KD.initDefault(0.0);
        break;
      case ROBOT_2K24_TEST:
        KS.initDefault(0.0);
        KV.initDefault(0.0);
        KP.initDefault(0.0);
        KD.initDefault(0.0);
        break;
      case ROBOT_SIM:
        KS.initDefault(0.0);
        KV.initDefault(0.037989);
        KP.initDefault(0.05);
        KD.initDefault(0.0);
        break;
      default:
        break;
    }
  }

  public Shooter(ShooterIO io) {
    this.io = io;

    feedforward = new SimpleMotorFeedforward(KS.get(), KV.get());
    feedback = new PIDController(KP.get(), 0.0, KD.get(), Constants.LOOP_PERIOD_SECS);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    // Adjust models based on tunable numbers
    if (KS.hasChanged(hashCode()) || KV.hasChanged(hashCode())) {
      feedforward = new SimpleMotorFeedforward(KS.get(), KV.get());
    }
    if (KP.hasChanged(hashCode())) {
      feedback.setP(KP.get());
    }
    if (KD.hasChanged(hashCode())) {
      feedback.setD(KD.get());
    }

    // Run control
    if (!isOpenLoop) {
      io.setVoltage(
          feedforward.calculate(feedback.getSetpoint())
              + feedback.calculate(inputs.velocityRadPerSec));
    } else {
      io.setVoltage(openLoopVoltage);
    }
  }

  private void setVelocity(double velocityRadPerSec) {
    isOpenLoop = false;
    feedback.setSetpoint(velocityRadPerSec);
  }

  private void stop() {
    isOpenLoop = true;
    openLoopVoltage = 0.0;
  }

  private void setVoltage(double volts) {
    isOpenLoop = true;
    openLoopVoltage = volts;
  }

  public Command runVelocity(double velocityRadPerSec) {
    return startEnd(
        () -> {
          setVelocity(velocityRadPerSec);
        },
        () -> {
          stop();
        });
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command runSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command runSysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
}
