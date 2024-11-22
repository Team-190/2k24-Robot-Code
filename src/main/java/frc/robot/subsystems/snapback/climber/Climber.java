package frc.robot.subsystems.snapback.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs;

  private final SysIdRoutine sysIdRoutineLeft;
  private final SysIdRoutine sysIdRoutineRight;

  public Climber(ClimberIO io) {
    this.io = io;
    inputs = new ClimberIOInputsAutoLogged();

    sysIdRoutineLeft =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(3.5),
                Seconds.of(10.0),
                (state) -> Logger.recordOutput("Climber/sysID State Left", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.setLeftVoltage(volts.in(Volts)), null, this));

    sysIdRoutineRight =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(3.5),
                Seconds.of(10.0),
                (state) -> Logger.recordOutput("Climber/sysID State Right", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.setRightVoltage(volts.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public boolean atGoal() {
    return io.atGoal();
  }

  public Command runSysId() {
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
