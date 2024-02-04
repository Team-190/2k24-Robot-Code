package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private static final double WHEEL_RADIUS = 0.0381;
  private static final LoggedTunableNumber voltage = new LoggedTunableNumber("Intake/voltage");
  private static final LoggedTunableNumber multiplier =
      new LoggedTunableNumber("Intake/multiplier");
  private static final LoggedTunableNumber nomagicnumbers =
      new LoggedTunableNumber("Intake/no magic numbers here");

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private Supplier<ChassisSpeeds> chassisSpeedsSupplier = () -> new ChassisSpeeds();
  private final SysIdRoutine sysIdRoutine;

  private static SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0);

  static {
    switch (Constants.ROBOT) {
      case ROBOT_2K24_C:
      case ROBOT_2K24_P:
        feedforward = new SimpleMotorFeedforward(0.13255, 0.034863);
        multiplier.initDefault(1.5);
        nomagicnumbers.initDefault(3);
        break;
      case ROBOT_2K24_TEST:
        feedforward = new SimpleMotorFeedforward(0.13255, 0.034863);
        multiplier.initDefault(1.5);
        nomagicnumbers.initDefault(3);
        break;
      case ROBOT_SIM:
        feedforward = new SimpleMotorFeedforward(0.13255, 0.034863);
        multiplier.initDefault(1.5);
        nomagicnumbers.initDefault(3);
        break;
      default:
        break;
    }
  }

  public Intake(IntakeIO io) {
    this.io = io;
    voltage.initDefault(0.0);

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Units.Volts.of(4.0),
                Units.Seconds.of(4.0),
                (state) -> Logger.recordOutput("Intake/sysIDState", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.setVoltage(volts.in(Volts)), null, this));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  private void stop() {
    io.setVoltage(0.0);
  }

  public void setChassisSpeedsSupplier(Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;
  }

  public Command runVoltage() {
    return runEnd(
        () -> {
          double volts =
              Math.max(
                  feedforward.calculate(
                      -chassisSpeedsSupplier.get().vxMetersPerSecond
                          * multiplier.get()
                          / WHEEL_RADIUS),
                  nomagicnumbers.get());
          Logger.recordOutput("Intake/Voltage", volts);
          io.setVoltage(volts);
        },
        () -> stop());
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
