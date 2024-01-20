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

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  private final Queue<Double> timestampQueue;

  private final StatusSignal<Double> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  private final StatusSignal<Double> turnAbsolutePosition;
  private final StatusSignal<Double> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnCurrent;

  // Gear ratios for SDS MK4i L2, adjust as necessary
  private final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  private final double TURN_GEAR_RATIO = 150.0 / 7.0;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOTalonFX(int index) {
    switch (Constants.ROBOT) {
      case ROBOT_2K24_C:
        switch (index) {
          case 0:
            driveTalon = new TalonFX(10);
            turnTalon = new TalonFX(11);
            cancoder = new CANcoder(12);
            absoluteEncoderOffset = Rotation2d.fromRadians(0.0); // TODO: Calibrate
            break;
          case 1:
            driveTalon = new TalonFX(20);
            turnTalon = new TalonFX(21);
            cancoder = new CANcoder(22);
            absoluteEncoderOffset = Rotation2d.fromRadians(0.0); // TODO: Calibrate
            break;
          case 2:
            driveTalon = new TalonFX(30);
            turnTalon = new TalonFX(31);
            cancoder = new CANcoder(32);
            absoluteEncoderOffset = Rotation2d.fromRadians(0.0); // TODO: Calibrate
            break;
          case 3:
            driveTalon = new TalonFX(40);
            turnTalon = new TalonFX(41);
            cancoder = new CANcoder(42);
            absoluteEncoderOffset = Rotation2d.fromRadians(0.0); // TODO: Calibrate
            break;
          default:
            throw new RuntimeException("Invalid module index");
        }
        break;
      case ROBOT_2K24_P:
        switch (index) {
          case 0:
            driveTalon = new TalonFX(10);
            turnTalon = new TalonFX(11);
            cancoder = new CANcoder(12);
            absoluteEncoderOffset = Rotation2d.fromRadians(0.0); // TODO: Calibrate
            break;
          case 1:
            driveTalon = new TalonFX(20);
            turnTalon = new TalonFX(21);
            cancoder = new CANcoder(22);
            absoluteEncoderOffset = Rotation2d.fromRadians(0.0); // TODO: Calibrate
            break;
          case 2:
            driveTalon = new TalonFX(30);
            turnTalon = new TalonFX(31);
            cancoder = new CANcoder(32);
            absoluteEncoderOffset = Rotation2d.fromRadians(0.0); // TODO: Calibrate
            break;
          case 3:
            driveTalon = new TalonFX(40);
            turnTalon = new TalonFX(41);
            cancoder = new CANcoder(42);
            absoluteEncoderOffset = Rotation2d.fromRadians(0.0); // TODO: Calibrate
            break;
          default:
            throw new RuntimeException("Invalid module index");
        }
        break;
      case ROBOT_2K24_TEST:
        switch (index) {
          case 0:
            driveTalon = new TalonFX(20, "drive");
            turnTalon = new TalonFX(21, "drive");
            cancoder = new CANcoder(22, "drive");
            absoluteEncoderOffset =
                Rotation2d.fromRadians(0.840621).minus(Rotation2d.fromDegrees(90));
            break;
          case 1:
            driveTalon = new TalonFX(10, "drive");
            turnTalon = new TalonFX(11, "drive");
            cancoder = new CANcoder(12, "drive");
            absoluteEncoderOffset =
                Rotation2d.fromRadians(-2.739689).plus(Rotation2d.fromDegrees(90));
            break;
          case 2:
            driveTalon = new TalonFX(40, "drive");
            turnTalon = new TalonFX(41, "drive");
            cancoder = new CANcoder(42, "drive");
            absoluteEncoderOffset =
                Rotation2d.fromRadians(1.469553).plus(Rotation2d.fromDegrees(90));
            ;
            break;
          case 3:
            driveTalon = new TalonFX(30, "drive");
            turnTalon = new TalonFX(31, "drive");
            cancoder = new CANcoder(32, "drive");
            absoluteEncoderOffset =
                Rotation2d.fromRadians(-1.325359).minus(Rotation2d.fromDegrees(90));
            ;
            break;
          default:
            throw new RuntimeException("Invalid module index");
        }
        break;
      default:
        throw new RuntimeException("Invalid robot");
    }

    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);

    var turnConfig = new TalonFXConfiguration();
    turnConfig.CurrentLimits.StatorCurrentLimit = 30.0;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnTalon.getConfigurator().apply(turnConfig);
    setTurnBrakeMode(true);

    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(turnTalon, turnTalon.getPosition());
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Module.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnPosition.getValueAsDouble() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsToRadians(turnVelocity.getValueAsDouble()) / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value) / DRIVE_GEAR_RATIO)
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value / TURN_GEAR_RATIO))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted =
        isTurnMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    turnTalon.getConfigurator().apply(config);
  }
}
