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
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public Rotation2d drivePosition = new Rotation2d();
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;
    public double driveTempCelcius = 0.0;

    public Rotation2d turnAbsolutePosition = new Rotation2d();
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;
    public double turnTempCelcius = 0.0;

    public double driveVelocitySetpointRadPerSec = 0.0;
    public Rotation2d turnPositionSetpoint = new Rotation2d();

    public double driveVelocityErrorRadPerSec = 0.0;
    public Rotation2d turnPositionError = new Rotation2d();

    public double[] odometryTimestamps = new double[] {};
    public Rotation2d[] odometryDrivePositionsRad = new Rotation2d[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  public default void updateInputs(ModuleIOInputs inputs) {}

  public default void setDriveVelocitySetpoint(
      double currentVelocityRadPerSec, double setpointVelocityRadsPerSec) {}

  public default void setTurnPositionSetpoint(
      Rotation2d currentPosition, Rotation2d setpointPosition) {}

  public default void setDriveVoltage(double volts) {}

  public default void setTurnVoltage(double volts) {}

  public default void setDrivePID(double kP, double kI, double kD) {}

  public default void setTurnPID(double kP, double kI, double kD) {}

  public default void setDriveFeedforward(double kS, double kV, double kA) {}

  public default void setDrivePosition(double position) {}

  public default void setTurnPosition(Rotation2d position) {}

  public default void stop() {}
}
