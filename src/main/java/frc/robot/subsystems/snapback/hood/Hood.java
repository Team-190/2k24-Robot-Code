package frc.robot.subsystems.snapback.hood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  public Hood(HoodIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }  
  
  public Command setVoltage(double volts) {
    return runOnce(() -> io.setVoltage(volts));
  }

  public Command setAmpSideFeed() {
    return runEnd(() -> io.setPositionSetpoint(Rotation2d.fromRotations(HoodConstants.AMP_SIDE_FEED_POSITION.get())), () -> io.setPositionSetpoint(Rotation2d.fromRotations(HoodConstants.STOWED_POSITION.get())));
  }

  public Command setSourceSideFeed() {
    return runEnd(() -> io.setPositionSetpoint(Rotation2d.fromRotations(HoodConstants.SOURCE_SIDE_FEED_POSITION.get())), () -> io.setPositionSetpoint(Rotation2d.fromRotations(HoodConstants.STOWED_POSITION.get())));
  }

  public Command setAmp() {
    return runEnd(() -> io.setPositionSetpoint(Rotation2d.fromRotations(HoodConstants.AMP_POSITION.get())), () -> io.setPositionSetpoint(Rotation2d.fromRotations(HoodConstants.STOWED_POSITION.get())));
  }

  public Command increaseAngle() {
    return Commands.runOnce(() -> HoodConstants.angleOffset += Units.degreesToRadians(0.25));
  }

    public Command decreaseAngle() {
    return Commands.runOnce(() -> HoodConstants.angleOffset -= Units.degreesToRadians(0.25));
  }
  
  // public Command setAnglePosition() {
  //   return runEnd(
  //     () -> {
  //       AimingParameters aimingParameters = ShotCalculator.angleCalculation();
  //       io.setPositionseSetpoint(Rotation2d.fromRadians(aimingParameters.shooterAngle().getRadians()));
  //     },
  //     () -> io.setPositionSetpoint(Rotation2d.fromRadians(HoodConstants.STOWED_POSITION.get())));
  // }

}
