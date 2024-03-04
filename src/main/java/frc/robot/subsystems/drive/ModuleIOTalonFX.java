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
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
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
  private final StatusSignal<Double> driveTemp;

  private final StatusSignal<Double> turnAbsolutePosition;
  private final StatusSignal<Double> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnCurrent;
  private final StatusSignal<Double> turnTemp;

  private final double DRIVE_GEAR_RATIO =
      (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0); // L3 Gearing
  private final double TURN_GEAR_RATIO = 150.0 / 7.0;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert cancoderDisconnectedAlert;

  public ModuleIOTalonFX(int index) {
    switch (Constants.ROBOT) {
      case SNAPBACK:
        switch (index) {
          case 0:
            driveTalon = new TalonFX(10);
            turnTalon = new TalonFX(11);
            cancoder = new CANcoder(20);
            absoluteEncoderOffset = Rotation2d.fromRadians(0.0); // TODO: Calibrate
            break;
          case 1:
            driveTalon = new TalonFX(8);
            turnTalon = new TalonFX(9);
            cancoder = new CANcoder(21);
            absoluteEncoderOffset = Rotation2d.fromRadians(0.0); // TODO: Calibrate
            break;
          case 2:
            driveTalon = new TalonFX(18);
            turnTalon = new TalonFX(19);
            cancoder = new CANcoder(22);
            absoluteEncoderOffset = Rotation2d.fromRadians(0.0); // TODO: Calibrate
            break;
          case 3:
            driveTalon = new TalonFX(0);
            turnTalon = new TalonFX(1);
            cancoder = new CANcoder(23);
            absoluteEncoderOffset = Rotation2d.fromRadians(0.0); // TODO: Calibrate
            break;
          default:
            throw new RuntimeException("Invalid module index");
        }
        break;
      case ROBOT_2K24_TEST:
        switch (index) {
          case 0:
            driveTalon = new TalonFX(0, "drive");
            turnTalon = new TalonFX(1, "drive");
            cancoder = new CANcoder(2, "drive");
            absoluteEncoderOffset = Rotation2d.fromRadians(-1.248660361338912);
            break;
          case 1:
            driveTalon = new TalonFX(10, "drive");
            turnTalon = new TalonFX(11, "drive");
            cancoder = new CANcoder(12, "drive");
            absoluteEncoderOffset = Rotation2d.fromRadians(-1.1090681096413186);
            break;
          case 2:
            driveTalon = new TalonFX(20, "drive");
            turnTalon = new TalonFX(21, "drive");
            cancoder = new CANcoder(22, "drive");
            absoluteEncoderOffset = Rotation2d.fromRadians(-1.9834371587361341);
            break;
          case 3:
            driveTalon = new TalonFX(30, "drive");
            turnTalon = new TalonFX(31, "drive");
            cancoder = new CANcoder(32, "drive");
            absoluteEncoderOffset = Rotation2d.fromRadians(0.07669903939428206);
            break;
          default:
            throw new RuntimeException("Invalid module index");
        }
        break;
      default:
        throw new RuntimeException("Invalid robot");
    }

    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);

    var turnConfig = new TalonFXConfiguration();
    turnConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turnTalon.getConfigurator().apply(turnConfig);
    setTurnBrakeMode(true);

    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getSupplyCurrent();
    driveTemp = driveTalon.getDeviceTemp();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(turnTalon, turnTalon.getPosition());
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getSupplyCurrent();
    turnTemp = turnTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Module.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        driveTemp,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent,
        turnTemp);
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();

    String moduleName =
        switch (index) {
          case 0 -> "FL";
          case 1 -> "FR";
          case 2 -> "BL";
          case 3 -> "BR";
          default -> "?";
        };
    driveDisconnectedAlert =
        new Alert(
            moduleName + " module drive Talon is disconnected, check CAN bus.", AlertType.ERROR);
    turnDisconnectedAlert =
        new Alert(
            moduleName + " module turn Talon is disconnected, check CAN bus.", AlertType.ERROR);
    cancoderDisconnectedAlert =
        new Alert(moduleName + " module CANcoder is disconnected, check CAN bus.", AlertType.ERROR);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    boolean driveConnected =
        BaseStatusSignal.refreshAll(driveVelocity, driveAppliedVolts, driveCurrent, driveTemp)
            .isOK();
    boolean turnConnected =
        BaseStatusSignal.refreshAll(
                turnPosition, turnVelocity, turnAppliedVolts, turnCurrent, turnTemp)
            .isOK();
    boolean cancoderConnected = BaseStatusSignal.refreshAll(turnAbsolutePosition).isOK();
    driveDisconnectedAlert.set(!driveConnected);
    turnDisconnectedAlert.set(!turnConnected);
    cancoderDisconnectedAlert.set(!cancoderConnected);

    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};
    inputs.driveTempCelcius = new double[] {driveTemp.getValueAsDouble()};

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnPosition.getValueAsDouble() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsToRadians(turnVelocity.getValueAsDouble()) / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};
    inputs.turnTempCelcius = new double[] {turnTemp.getValueAsDouble()};

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
