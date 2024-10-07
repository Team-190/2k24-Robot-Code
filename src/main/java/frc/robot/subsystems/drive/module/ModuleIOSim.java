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

package frc.robot.subsystems.drive.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ModuleIOSim implements ModuleIO {
  private DCMotorSim driveMotorSim;
  private DCMotorSim turnMotorSim;

  private SimpleMotorFeedforward driveFeedforward;

  private final PIDController driveFeedback;
  private final PIDController turnFeedback;

  private final Rotation2d turnAbsoluteInitPosition;

  private double driveAppliedVolts;
  private double turnAppliedVolts;

  public ModuleIOSim() {
    driveMotorSim =
        new DCMotorSim(
            ModuleConstants.DRIVE_MOTOR_CONFIG,
            ModuleConstants.DRIVE_GEAR_RATIO,
            ModuleConstants.DRIVE_MOMENT_OF_INERTIA);
    turnMotorSim =
        new DCMotorSim(
            ModuleConstants.TURN_MOTOR_CONFIG,
            ModuleConstants.TURN_GEAR_RATIO,
            ModuleConstants.TURN_MOMENT_OF_INERTIA);

    driveFeedforward =
        new SimpleMotorFeedforward(ModuleConstants.DRIVE_KS.get(), ModuleConstants.DRIVE_KV.get());

    driveFeedback =
        new PIDController(
            ModuleConstants.DRIVE_KP.get(),
            0.0,
            ModuleConstants.DRIVE_KD.get(),
            Constants.LOOP_PERIOD_SECONDS);
    turnFeedback =
        new PIDController(
            ModuleConstants.TURN_KP.get(),
            0.0,
            ModuleConstants.TURN_KD.get(),
            Constants.LOOP_PERIOD_SECONDS);
    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);

    turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);

    driveAppliedVolts = 0.0;
    turnAppliedVolts = 0.0;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    driveMotorSim.update(Constants.LOOP_PERIOD_SECONDS);
    turnMotorSim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new Rotation2d[] {inputs.drivePosition};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};

    inputs.drivePosition = Rotation2d.fromRadians(driveMotorSim.getAngularPositionRad());
    inputs.driveVelocityRadPerSec = driveMotorSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(driveMotorSim.getCurrentDrawAmps());

    inputs.turnAbsolutePosition =
        new Rotation2d(turnMotorSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
    inputs.turnPosition = new Rotation2d(turnMotorSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnMotorSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(turnMotorSim.getCurrentDrawAmps());

    inputs.driveVelocitySetpointRadPerSec = driveFeedback.getSetpoint();
    inputs.turnPositionSetpoint = Rotation2d.fromRadians(turnFeedback.getSetpoint());

    inputs.driveVelocityErrorRadPerSec = driveFeedback.getPositionError();
    inputs.turnPositionError = Rotation2d.fromRadians(turnFeedback.getPositionError());
  }

  @Override
  public void setDriveVelocitySetpoint(
      double currentVelocityRadPerSec, double setpointVelocityRadPerSec) {
    driveAppliedVolts =
        MathUtil.clamp(
            driveFeedback.calculate(currentVelocityRadPerSec, setpointVelocityRadPerSec)
                + driveFeedforward.calculate(setpointVelocityRadPerSec),
            -12.0,
            12.0);
    driveMotorSim.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setTurnPositionSetpoint(Rotation2d currentPosition, Rotation2d setpointPosition) {
    turnAppliedVolts =
        MathUtil.clamp(
            turnFeedback.calculate(currentPosition.getRadians(), setpointPosition.getRadians()),
            -12.0,
            12.0);
    turnMotorSim.setInputVoltage(turnAppliedVolts);
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveMotorSim.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    turnMotorSim.setInputVoltage(turnAppliedVolts);
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveFeedback.setPID(kP, kI, kD);
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    turnFeedback.setPID(kP, kI, kD);
  }

  @Override
  public void setDriveFeedforward(double kS, double kV, double kA) {
    driveFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
  }

  @Override
  public void stop() {
    driveAppliedVolts = 0.0;
    turnAppliedVolts = 0.0;

    driveMotorSim.setInputVoltage(0.0);
    turnMotorSim.setInputVoltage(0.0);
  }
}
