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
import frc.robot.util.LinearProfile;
import frc.robot.util.LoggedTunableNumber;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private static final LoggedTunableNumber RATIO = new LoggedTunableNumber("Shooter/Ratio");

  private static final LoggedTunableNumber KP = new LoggedTunableNumber("Shooter/Kp");
  private static final LoggedTunableNumber KD = new LoggedTunableNumber("Shooter/Kd");

  private static final LoggedTunableNumber KS_LEFT = new LoggedTunableNumber("Shooter/Left Ks");
  private static final LoggedTunableNumber KV_LEFT = new LoggedTunableNumber("Shooter/Left Kv");
  private static final LoggedTunableNumber KA_LEFT = new LoggedTunableNumber("Shooter/Left Ka");

  private static final LoggedTunableNumber KS_RIGHT = new LoggedTunableNumber("Shooter/Right Ks");
  private static final LoggedTunableNumber KV_RIGHT = new LoggedTunableNumber("Shooter/Right Kv");
  private static final LoggedTunableNumber KA_RIGHT = new LoggedTunableNumber("Shooter/Right Ka");

  private static final LoggedTunableNumber MAX_ACCELERATION =
      new LoggedTunableNumber("Shooter/Max Acceleration");

  private static final LoggedTunableNumber DEFAULT_SPEED =
      new LoggedTunableNumber("Shooter/Default Speed");

  private static final LoggedTunableNumber AMP_SPEED = new LoggedTunableNumber("Shooter/Amp Speed");

  private static final LoggedTunableNumber GOAL_TOLERANCE =
      new LoggedTunableNumber("Shooter/Goal Tolerance");

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private static SimpleMotorFeedforward leftFeedforward;
  private static SimpleMotorFeedforward rightFeedforward;

  private static LinearProfile leftProfile;
  private static LinearProfile rightProfile;

  private static PIDController leftFeedback;
  private static PIDController rightFeedback;

  private boolean isOpenLoop = true;
  private double openLoopVoltage = 0.0;

  private double leftGoal = 0.0;
  private double rightGoal = 0.0;

  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.2).per(Seconds.of(1.0)),
              Volts.of(3.5),
              Seconds.of(10.0),
              (state) -> Logger.recordOutput("Shooter/sysID State", state.toString())),
          new SysIdRoutine.Mechanism((volts) -> setVoltage(volts.in(Volts)), null, this));

  private static enum SpinDirection {
    COUNTERCLOCKWISE,
    CLOCKWISE,
    YEET
  }

  @AutoLogOutput private SpinDirection spinDirection = SpinDirection.CLOCKWISE;

  private double flywheelOffset = 0;
  private double spinOffset = 0;

  private boolean isShooting = false;

  static {
    GOAL_TOLERANCE.initDefault(10);
    KS_LEFT.initDefault(0.134);
    KV_LEFT.initDefault(0.0071266);
    KA_LEFT.initDefault(0.0);
    KS_RIGHT.initDefault(0.14543);
    KV_RIGHT.initDefault(0.0068754);
    KA_RIGHT.initDefault(0.0);
    switch (Constants.ROBOT) {
      case SNAPBACK:
        KP.initDefault(0.008);
        KD.initDefault(0.0);
        RATIO.initDefault(0.5);
        DEFAULT_SPEED.initDefault(600);
        AMP_SPEED.initDefault(550);
        break;
      case ROBOT_2K24_TEST:
        KP.initDefault(0.035);
        KD.initDefault(0.0);
        RATIO.initDefault(2.0 / 3.0);
        DEFAULT_SPEED.initDefault(400);
        AMP_SPEED.initDefault(0.0);
        break;
      case ROBOT_SIM:
        KP.initDefault(0.035);
        KD.initDefault(0.0);
        RATIO.initDefault(2.0 / 3.0);
        DEFAULT_SPEED.initDefault(600);
        AMP_SPEED.initDefault(0.0);
        break;
      default:
        break;
    }
  }

  public Shooter(ShooterIO io) {
    this.io = io;
    leftFeedback = new PIDController(KP.get(), 0.0, KD.get(), Constants.LOOP_PERIOD_SECS);
    rightFeedback = new PIDController(KP.get(), 0.0, KD.get(), Constants.LOOP_PERIOD_SECS);

    leftFeedforward = new SimpleMotorFeedforward(KS_LEFT.get(), KV_LEFT.get(), KA_LEFT.get());
    rightFeedforward = new SimpleMotorFeedforward(KS_RIGHT.get(), KV_RIGHT.get(), KA_RIGHT.get());

    leftProfile = new LinearProfile(MAX_ACCELERATION.get(), Constants.LOOP_PERIOD_SECS);
    rightProfile = new LinearProfile(MAX_ACCELERATION.get(), Constants.LOOP_PERIOD_SECS);

    setDefaultCommand(runVelocity());
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
    if (MAX_ACCELERATION.hasChanged(hashCode())) {
      leftProfile.setMaxAcceleration(MAX_ACCELERATION.get());
      rightProfile.setMaxAcceleration(MAX_ACCELERATION.get());
    }
    if (KS_LEFT.hasChanged(hashCode())
        || KV_LEFT.hasChanged(hashCode())
        || KA_LEFT.hasChanged(hashCode())) {
      leftFeedforward = new SimpleMotorFeedforward(KS_LEFT.get(), KV_LEFT.get(), KA_LEFT.get());
    }
    if (KS_RIGHT.hasChanged(hashCode())
        || KV_RIGHT.hasChanged(hashCode())
        || KA_RIGHT.hasChanged(hashCode())) {
      rightFeedforward = new SimpleMotorFeedforward(KS_RIGHT.get(), KV_RIGHT.get(), KA_RIGHT.get());
    }

    if (!isOpenLoop) {
      leftProfile.setGoal(leftGoal);
      rightProfile.setGoal(rightGoal);
      double leftSetpoint = leftProfile.calculateSetpoint();
      double rightSetpoint = rightProfile.calculateSetpoint();
      io.setLeftVoltage(
          leftFeedforward.calculate(leftSetpoint)
              + leftFeedback.calculate(inputs.leftVelocityRadPerSec));
      io.setRightVoltage(
          rightFeedforward.calculate(rightSetpoint)
              + rightFeedback.calculate(inputs.rightVelocityRadPerSec));
    } else {
      io.setLeftVoltage(openLoopVoltage);
      io.setRightVoltage(openLoopVoltage);
    }

    Logger.recordOutput("Shooter/Left Goal", leftProfile.getGoal());
    Logger.recordOutput("Shooter/Right Goal", rightProfile.getGoal());
  }

  private void setVelocity(double velocityRadPerSec) {
    isOpenLoop = false;
    if (spinDirection.equals(SpinDirection.COUNTERCLOCKWISE)) {
      leftProfile.setGoal((velocityRadPerSec + flywheelOffset) * (RATIO.get() + spinOffset));
      rightProfile.setGoal(velocityRadPerSec + flywheelOffset);
    } else if (spinDirection.equals(SpinDirection.CLOCKWISE)) {
      leftProfile.setGoal(velocityRadPerSec + flywheelOffset);
      rightProfile.setGoal((velocityRadPerSec + flywheelOffset) * (RATIO.get() + spinOffset));
    } else {
      // YEET MODE :)
      leftProfile.setGoal(velocityRadPerSec + flywheelOffset);
      rightProfile.setGoal(velocityRadPerSec + flywheelOffset);
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

  public double getSpeed() {
    return Math.max(inputs.leftVelocityRadPerSec, inputs.rightVelocityRadPerSec);
  }

  public double getFlywheelOffset() {
    return flywheelOffset;
  }

  public double getSpinOffset() {
    return spinOffset;
  }

  public boolean isShooting() {
    return isShooting;
  }

  public boolean atGoal() {
    return (Math.abs(leftProfile.getGoal() - leftFeedback.getSetpoint()) <= GOAL_TOLERANCE.get())
        && (Math.abs(rightProfile.getGoal() - rightFeedback.getSetpoint()) <= GOAL_TOLERANCE.get());
  }

  public Command runVelocity() {
    return runEnd(
        () -> {
          setVelocity(DEFAULT_SPEED.get());
        },
        () -> {
          stop();
        });
  }

  public Command runAmp() {
    return runEnd(
        () -> {
          setVelocity(AMP_SPEED.get());
        },
        () -> {
          stop();
        });
  }

  public Command runPoseDistance(
      Supplier<Optional<Translation2d>> robotPoseSupplier,
      Supplier<Translation2d> velocitySupplier) {
    return runEnd(
        () -> {
          if (robotPoseSupplier.get().isPresent()) {
            AimingParameters aimingParameters =
                ShotCalculator.poseCalculation(
                    robotPoseSupplier.get().get(), velocitySupplier.get());
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
                ShotCalculator.poseCalculation(
                        robotPoseSupplier.get().get(), velocitySupplier.get())
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

  public Command runAngleDistance() {
    return runEnd(
        () -> {
          setVelocity(ShotCalculator.angleCalculation().shooterSpeed());
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

  public Command increaseVelocity() {
    return Commands.runOnce(() -> flywheelOffset += 10);
  }

  public Command decreaseVelocity() {
    return Commands.runOnce(() -> flywheelOffset -= 10);
  }

  public Command increaseSpin() {
    return Commands.runOnce(
        () -> {
          if (RATIO.get() + spinOffset < 0.9) spinOffset += 0.1;
        });
  }

  public Command decreaseSpin() {
    return Commands.runOnce(
        () -> {
          if (RATIO.get() + spinOffset > 0.1) spinOffset -= 0.1;
        });
  }
}
