package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private static final LoggedTunableNumber RATIO = new LoggedTunableNumber("Shooter/Ratio");

  private static final LoggedTunableNumber KP = new LoggedTunableNumber("Shooter/Kp");
  private static final LoggedTunableNumber KD = new LoggedTunableNumber("Shooter/Kd");

  private static final double LEFT_KS = 0.0;
  private static final double LEFT_KV = 0.0;

  private static final double RIGHT_KS = 0.0;
  private static final double RIGHT_KV = 0.0;

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private static final InterpolatingDoubleTreeMap speakerDistanceToShooterSpeed =
      new InterpolatingDoubleTreeMap();

  private SimpleMotorFeedforward leftFeedforward;
  private SimpleMotorFeedforward rightFeedforward;

  private final PIDController feedback;
  private boolean isOpenLoop = true;
  private double openLoopVoltage = 0.0;
  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              null,
              Seconds.of(6.0),
              (state) -> Logger.recordOutput("Shooter/sysIDState", state.toString())),
          new SysIdRoutine.Mechanism((volts) -> setVoltage(volts.in(Volts)), null, this));

  static {
    switch (Constants.ROBOT) {
      case ROBOT_2K24_C:
      case ROBOT_2K24_P:
        KP.initDefault(0.0);
        KD.initDefault(0.0);
        RATIO.initDefault(0.0);
        break;
      case ROBOT_2K24_TEST:
        KP.initDefault(0.05);
        KD.initDefault(0.0);
        RATIO.initDefault(0.0);
        break;
      case ROBOT_SIM:
        KP.initDefault(0.05);
        KD.initDefault(0.0);
        RATIO.initDefault(0.0);
        break;
      default:
        break;
    }

    speakerDistanceToShooterSpeed.put(Units.inchesToMeters(60), 10.0);
    speakerDistanceToShooterSpeed.put(Units.inchesToMeters(80), 17.0);
    speakerDistanceToShooterSpeed.put(Units.inchesToMeters(100), 9.0);
    speakerDistanceToShooterSpeed.put(Units.inchesToMeters(120), 30.0);
    speakerDistanceToShooterSpeed.put(Units.inchesToMeters(140), 19.0);
    speakerDistanceToShooterSpeed.put(Units.inchesToMeters(160), 88.0);
    speakerDistanceToShooterSpeed.put(Units.inchesToMeters(180), 10.0);
  }

  public Shooter(ShooterIO io) {
    this.io = io;

    leftFeedforward = new SimpleMotorFeedforward(LEFT_KS, LEFT_KV);
    rightFeedforward = new SimpleMotorFeedforward(RIGHT_KS, RIGHT_KV);

    feedback = new PIDController(KP.get(), 0.0, KD.get(), Constants.LOOP_PERIOD_SECS);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    // Adjust models based on tunable numbers
    if (KP.hasChanged(hashCode())) {
      feedback.setP(KP.get());
    }
    if (KD.hasChanged(hashCode())) {
      feedback.setD(KD.get());
    }

    // Run control
    if (!isOpenLoop) {
      io.setLeftVoltage(
          leftFeedforward.calculate(feedback.getSetpoint())
              + feedback.calculate(inputs.leftVelocityRadPerSec));
      io.setRightVoltage(
          rightFeedforward.calculate(feedback.getSetpoint())
              + feedback.calculate(inputs.rightVelocityRadPerSec));
    } else {
      io.setLeftVoltage(openLoopVoltage);
      io.setRightVoltage(openLoopVoltage);
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

  public Command runDistance(Supplier<Optional<Double>> getSpeakerDistance) {
    return runEnd(
        () -> {
          Optional<Double> distOptional = getSpeakerDistance.get();
          if (distOptional.isPresent())
            setVelocity(speakerDistanceToShooterSpeed.get(distOptional.get()));
        },
        () -> {
          stop();
        });
  }

  public Command runSysId() {
    return Commands.sequence(
        sysIdRoutine.quasistatic(Direction.kForward),
        Commands.waitSeconds(6),
        sysIdRoutine.quasistatic(Direction.kReverse),
        Commands.waitSeconds(6),
        sysIdRoutine.dynamic(Direction.kForward),
        Commands.waitSeconds(6),
        sysIdRoutine.dynamic(Direction.kReverse));
  }
}
