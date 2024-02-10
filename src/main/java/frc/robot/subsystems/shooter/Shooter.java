package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private static final LoggedTunableNumber RATIO = new LoggedTunableNumber("Shooter/Ratio");

  private static final LoggedTunableNumber KP = new LoggedTunableNumber("Shooter/Kp");
  private static final LoggedTunableNumber KD = new LoggedTunableNumber("Shooter/Kd");

  private static final LoggedTunableNumber SPEED = new LoggedTunableNumber("Shooter/Speed");

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private static SimpleMotorFeedforward leftFeedforward;
  private static SimpleMotorFeedforward rightFeedforward;

  private boolean isShooting = false;

  private final PIDController leftFeedback;
  private final PIDController rightFeedback;
  private boolean isOpenLoop = true;
  private double openLoopVoltage = 0.0;
  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.2).per(Seconds.of(1.0)),
              Volts.of(3.5),
              Seconds.of(180.0),
              (state) -> Logger.recordOutput("Shooter/sysIDState", state.toString())),
          new SysIdRoutine.Mechanism((volts) -> setVoltage(volts.in(Volts)), null, this));

  static {
    switch (Constants.ROBOT) {
      case ROBOT_2K24_C:
      case ROBOT_2K24_P:
        KP.initDefault(0.035);
        KD.initDefault(0.0);
        RATIO.initDefault(2.0 / 3.0);
        SPEED.initDefault(400);
        leftFeedforward = new SimpleMotorFeedforward(0.49147, 0.0069249);
        rightFeedforward = new SimpleMotorFeedforward(0.72165, 0.0075142);
        break;
      case ROBOT_2K24_TEST:
        KP.initDefault(0.035);
        KD.initDefault(0.0);
        RATIO.initDefault(2.0 / 3.0);
        SPEED.initDefault(400);

        leftFeedforward = new SimpleMotorFeedforward(0.49147, 0.0069249);
        rightFeedforward = new SimpleMotorFeedforward(0.72165, 0.0075142);
        break;
      case ROBOT_SIM:
        KP.initDefault(0.035);
        KD.initDefault(0.0);
        RATIO.initDefault(2.0 / 3.0);
        SPEED.initDefault(400);

        leftFeedforward = new SimpleMotorFeedforward(0.49147, 0.0069249);
        rightFeedforward = new SimpleMotorFeedforward(0.72165, 0.0075142);
        break;
      default:
        break;
    }
  }

  public Shooter(ShooterIO io) {
    this.io = io;
    leftFeedback = new PIDController(KP.get(), 0.0, KD.get(), Constants.LOOP_PERIOD_SECS);
    rightFeedback = new PIDController(KP.get(), 0.0, KD.get(), Constants.LOOP_PERIOD_SECS);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    // Adjust models based on tunable numbers
    if (KP.hasChanged(hashCode())) {
      leftFeedback.setP(KP.get());
      rightFeedback.setP(KP.get());
    }
    if (KD.hasChanged(hashCode())) {
      leftFeedback.setD(KD.get());
      rightFeedback.setD(KD.get());
    }

    // Run control
    if (!isOpenLoop) {
      io.setLeftVoltage(
          leftFeedforward.calculate(leftFeedback.getSetpoint())
              + leftFeedback.calculate(inputs.leftVelocityRadPerSec));
      io.setRightVoltage(
          rightFeedforward.calculate(rightFeedback.getSetpoint())
              + rightFeedback.calculate(inputs.rightVelocityRadPerSec));
    } else {
      io.setLeftVoltage(openLoopVoltage);
      io.setRightVoltage(openLoopVoltage);
    }
  }

  private void setVelocity(double velocityRadPerSec) {
    isOpenLoop = false;
    leftFeedback.setSetpoint(velocityRadPerSec);
    rightFeedback.setSetpoint(velocityRadPerSec * RATIO.get());
  }

  private void stop() {
    isOpenLoop = true;
    openLoopVoltage = 0.0;
  }

  private void setVoltage(double volts) {
    isOpenLoop = true;
    openLoopVoltage = volts;
  }

  public boolean isShooting() {
    return isShooting;
  }

  public Command runVelocity() {
    return runEnd(
        () -> {
          setVelocity(SPEED.get());
          isShooting = true;
        },
        () -> {
          stop();
          isShooting = false;
        });
  }

  // public Command runDistance(Supplier<Optional<Double>> getSpeakerDistance) {
  //   return runEnd(
  //       () -> {
  //         Optional<Double> distOptional = getSpeakerDistance.get();
  //         if (distOptional.isPresent())
  //           setVelocity(speakerDistanceToShooterSpeed.get(distOptional.get()));
  //       },
  //       () -> {
  //         stop();
  //       });
  // }

  public Command runSysId() {
    return Commands.sequence(
        sysIdRoutine.quasistatic(Direction.kForward),
        Commands.waitSeconds(4),
        sysIdRoutine.quasistatic(Direction.kReverse),
        Commands.waitSeconds(4),
        sysIdRoutine.dynamic(Direction.kForward),
        Commands.waitSeconds(4),
        sysIdRoutine.dynamic(Direction.kReverse));
  }
}
