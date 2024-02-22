package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.ShotCalculator;
import frc.robot.ShotCalculator.AimingParameters;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private static final LoggedTunableNumber DIFFERENCE =
      new LoggedTunableNumber("Shooter/Difference");

  private static final LoggedTunableNumber KP = new LoggedTunableNumber("Shooter/Kp");
  private static final LoggedTunableNumber KD = new LoggedTunableNumber("Shooter/Kd");

  private static final LoggedTunableNumber DEFAULT_SPEED =
      new LoggedTunableNumber("Shooter/Default Speed");

  private static final LoggedTunableNumber AMP_SPEED = new LoggedTunableNumber("Shooter/Amp Speed");

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
              (state) -> Logger.recordOutput("Shooter/sysID State", state.toString())),
          new SysIdRoutine.Mechanism((volts) -> setVoltage(volts.in(Volts)), null, this));

  private static enum SpinDirection {
    COUNTERCLOCKWISE,
    CLOCKWISE,
    YEET
  }

  @AutoLogOutput private SpinDirection spinDirection = SpinDirection.CLOCKWISE;

  static {
    switch (Constants.ROBOT) {
      case ROBOT_2K24_C:
      case ROBOT_2K24_P:
        KP.initDefault(0.035);
        KD.initDefault(0.0);
        DIFFERENCE.initDefault(2.0 / 3.0);
        DEFAULT_SPEED.initDefault(400);
        leftFeedforward = new SimpleMotorFeedforward(0.49147, 0.0069249);
        rightFeedforward = new SimpleMotorFeedforward(0.72165, 0.0075142);
        break;
      case ROBOT_2K24_TEST:
        KP.initDefault(0.035);
        KD.initDefault(0.0);
        DIFFERENCE.initDefault(2.0 / 3.0);
        DEFAULT_SPEED.initDefault(400);

        leftFeedforward = new SimpleMotorFeedforward(0.49147, 0.0069249);
        rightFeedforward = new SimpleMotorFeedforward(0.72165, 0.0075142);
        break;
      case ROBOT_SIM:
        KP.initDefault(0.035);
        KD.initDefault(0.0);
        DIFFERENCE.initDefault(2.0 / 3.0);
        DEFAULT_SPEED.initDefault(400);

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

    if (KP.hasChanged(hashCode())) {
      leftFeedback.setP(KP.get());
      rightFeedback.setP(KP.get());
    }
    if (KD.hasChanged(hashCode())) {
      leftFeedback.setD(KD.get());
      rightFeedback.setD(KD.get());
    }

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
    if (spinDirection.equals(SpinDirection.COUNTERCLOCKWISE)) {
      leftFeedback.setSetpoint(velocityRadPerSec - DIFFERENCE.get());
      rightFeedback.setSetpoint(velocityRadPerSec);
    } else if (spinDirection.equals(SpinDirection.CLOCKWISE)) {
      leftFeedback.setSetpoint(velocityRadPerSec);
      rightFeedback.setSetpoint(velocityRadPerSec - DIFFERENCE.get());
    } else {
      leftFeedback.setSetpoint(velocityRadPerSec);
      rightFeedback.setSetpoint(velocityRadPerSec);
    }
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
          setVelocity(DEFAULT_SPEED.get());
          isShooting = true;
        },
        () -> {
          stop();
          isShooting = false;
        });
  }

  public Command runAmp() {
    return runEnd(
        () -> {
          setVelocity(AMP_SPEED.get());
          isShooting = true;
        },
        () -> {
          stop();
          isShooting = false;
        });
  }

  public Command runDistance(
      Supplier<Optional<Translation2d>> robotPoseSupplier,
      Supplier<Translation2d> velocitySupplier) {
    return runEnd(
        () -> {
          if (robotPoseSupplier.get().isPresent()) {
            AimingParameters aimingParameters =
                ShotCalculator.calculate(robotPoseSupplier.get().get(), velocitySupplier.get());
            if (spinDirection.equals(SpinDirection.YEET)) {
              spinDirection = SpinDirection.CLOCKWISE;
            }
            Rotation2d effectiveRobotAngle = aimingParameters.robotAngle();

            Translation2d speakerPose =
                AllianceFlipUtil.apply(
                    FieldConstants.Speaker.centerSpeakerOpening.getTranslation());
            double distanceToSpeaker = robotPoseSupplier.get().get().getDistance(speakerPose);

            if (distanceToSpeaker > 1.5) {
              if (DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get().equals(Alliance.Blue)) {
                effectiveRobotAngle = effectiveRobotAngle.minus(Rotation2d.fromDegrees(180));
              }
              if (effectiveRobotAngle.getDegrees() > 15) {
                spinDirection = SpinDirection.COUNTERCLOCKWISE;
              } else if (effectiveRobotAngle.getDegrees() < -15) {
                spinDirection = SpinDirection.CLOCKWISE;
              }
            }

            setVelocity(
                ShotCalculator.calculate(robotPoseSupplier.get().get(), velocitySupplier.get())
                    .shooterSpeed());
          } else {
            if (DriverStation.isAutonomous()) {
              setVelocity(DEFAULT_SPEED.get());
            }
          }
        },
        () -> {
          stop();
        });
  }

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
