// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.shared.drive.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIOInputsAutoLogged inputs;

  private SwerveModulePosition[] odometryPositions;
  private Rotation2d angleSetpoint;
  private Double speedSetpoint;
  private Rotation2d turnRelativeOffset;

  private final ModuleIO io;
  private final int index;

  public Module(ModuleIO io, int index) {
    inputs = new ModuleIOInputsAutoLogged();

    odometryPositions = new SwerveModulePosition[] {};
    angleSetpoint = null;
    speedSetpoint = null;
    turnRelativeOffset = null;

    this.io = io;
    this.index = index;
  }

  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public void periodic() {
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Adjust models based on tunable numbers
    double startTunableCheck = System.currentTimeMillis();
    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> io.setDrivePID(pid[0], 0.0, pid[1]),
        ModuleConstants.DRIVE_KP,
        ModuleConstants.DRIVE_KD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> io.setTurnPID(pid[0], 0.0, pid[1]),
        ModuleConstants.TURN_KP,
        ModuleConstants.TURN_KD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        ff -> io.setDriveFeedforward(ff[0], ff[1], 0.0),
        ModuleConstants.DRIVE_KS,
        ModuleConstants.DRIVE_KV);

    double startSetPosition = System.currentTimeMillis();
    if (turnRelativeOffset == null
        && inputs.turnAbsolutePosition.getRadians() != 0.0
        && inputs.turnPosition.getRadians() != 0.0) {
      turnRelativeOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition);
      io.setTurnPosition(inputs.turnAbsolutePosition);
      io.setDrivePosition(0.0);
    }
    double endSetPosition = System.currentTimeMillis();

    if (angleSetpoint != null && DriverStation.isEnabled()) {
      io.setTurnPositionSetpoint(inputs.turnAbsolutePosition, angleSetpoint);

      if (speedSetpoint != null) {
        double adjustSpeedSetpoint = speedSetpoint * inputs.turnPositionError.getCos();

        double velocityRadPerSec = adjustSpeedSetpoint / ModuleConstants.WHEEL_RADIUS.get();
        io.setDriveVelocitySetpoint(inputs.driveVelocityRadPerSec, velocityRadPerSec);
      }
    }
    double endRunIO = System.currentTimeMillis();

    int sampleCount = inputs.odometryTimestamps.length;
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters =
          inputs.odometryDrivePositionsRad[i].getRadians() * ModuleConstants.WHEEL_RADIUS.get();
      Rotation2d angle =
          inputs.odometryTurnPositions[i].plus(
              turnRelativeOffset != null ? turnRelativeOffset : new Rotation2d());
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }
    double endUpdateOdom = System.currentTimeMillis();

    Logger.recordOutput(
        "Drive/Time/Module " + index + "/Change Constants", startSetPosition - startTunableCheck);
    Logger.recordOutput(
        "Drive/Time/Module " + index + "/Set Position", endSetPosition - startSetPosition);
    Logger.recordOutput("Drive/Time/Module " + index + "/Run IO", endRunIO - endSetPosition);
    Logger.recordOutput(
        "Drive/Time/Module " + index + "/Odometry Update", endUpdateOdom - endRunIO);
    Logger.recordOutput(
        "Drive/Module" + Integer.toString(index) + "/position", inputs.drivePosition.getRadians());
  }

  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    var optimizedState = SwerveModuleState.optimize(state, inputs.turnAbsolutePosition);

    angleSetpoint = optimizedState.angle;
    speedSetpoint = optimizedState.speedMetersPerSecond;

    return optimizedState;
  }

  public void runCharacterization(double volts) {
    angleSetpoint = new Rotation2d();

    io.setDriveVoltage(volts);
    speedSetpoint = null;
  }

  public void stop() {
    io.stop();

    angleSetpoint = null;
    speedSetpoint = null;
  }

  public double getPositionMeters() {
    return inputs.drivePosition.getRadians() * ModuleConstants.WHEEL_RADIUS.get();
  }

  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * ModuleConstants.WHEEL_RADIUS.get();
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), inputs.turnAbsolutePosition);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), inputs.turnAbsolutePosition);
  }

  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  public double getCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }
}
