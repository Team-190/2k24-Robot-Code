package frc.robot.subsystems.snapback.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs;

  private SysIdRoutine characterizationRoutine;

  public Hood(HoodIO io) {
    inputs = new HoodIOInputsAutoLogged();
    this.io = io;

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Seconds),
                Volts.of(3.5),
                Seconds.of(10),
                (state) -> Logger.recordOutput("Arm/sysIDState", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.setVoltage(volts.in(Volts)), null, this));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }

  public Command setVoltage(double volts) {
    return runOnce(() -> io.setVoltage(volts));
  }

  public Command setAmpSideFeed() {
    return runEnd(
        () ->
            io.setPositionSetpoint(
                Rotation2d.fromRotations(HoodConstants.AMP_SIDE_FEED_POSITION.get())),
        () ->
            io.setPositionSetpoint(Rotation2d.fromRotations(HoodConstants.STOWED_POSITION.get())));
  }

  public Command setSourceSideFeed() {
    return runEnd(
        () ->
            io.setPositionSetpoint(
                Rotation2d.fromRotations(HoodConstants.SOURCE_SIDE_FEED_POSITION.get())),
        () ->
            io.setPositionSetpoint(Rotation2d.fromRotations(HoodConstants.STOWED_POSITION.get())));
  }

  public Command setAmp() {
    return runEnd(
        () -> io.setPositionSetpoint(Rotation2d.fromRotations(HoodConstants.AMP_POSITION.get())),
        () ->
            io.setPositionSetpoint(Rotation2d.fromRotations(HoodConstants.STOWED_POSITION.get())));
  }

  public Command increaseAngle() {
    return Commands.runOnce(() -> HoodConstants.angleOffset += Units.degreesToRadians(0.25));
  }

  public Command decreaseAngle() {
    return Commands.runOnce(() -> HoodConstants.angleOffset -= Units.degreesToRadians(0.25));
  }

  // public Command setAnglePosition() {
  // return runEnd(
  // () -> {
  // AimingParameters aimingParameters = ShotCalculator.angleCalculation();
  // io.setPositionseSetpoint(Rotation2d.fromRadians(aimingParameters.shooterAngle().getRadians()));
  // },
  // () ->
  // io.setPositionSetpoint(Rotation2d.fromRadians(HoodConstants.STOWED_POSITION.get())));
  // }

  public Command runSysId() {
    return Commands.sequence(
      characterizationRoutine.quasistatic(Direction.kForward),
      Commands.waitSeconds(3),
      characterizationRoutine.quasistatic(Direction.kReverse),
      Commands.waitSeconds(3),
      characterizationRoutine.dynamic(Direction.kForward),
      Commands.waitSeconds(3),
      characterizationRoutine.dynamic(Direction.kReverse)
    );
  }

}
