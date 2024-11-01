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

package frc.robot.subsystems.shared.drive.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.subsystems.shared.drive.drive.DriveConstants;
import frc.robot.subsystems.shared.drive.drive.PhoenixOdometryThread;
import frc.robot.subsystems.shared.drive.module.ModuleConstants;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import java.util.Queue;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon =
      new Pigeon2(DriveConstants.PIGEON_2_DEVICE_ID, DriveConstants.CANIVORE);

  private final StatusSignal<Double> yaw;
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<Double> yawVelocity;

  private final Alert disconnectedAlert =
      new Alert("Pigeon is disconnected, check CAN bus.", AlertType.ERROR);

  public GyroIOPigeon2() {
    yaw = pigeon.getYaw();
    yawVelocity = pigeon.getAngularVelocityZWorld();
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(ModuleConstants.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(100.0);
    pigeon.optimizeBusUtilization();

    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon, pigeon.getYaw());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    boolean connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).isOK();
    disconnectedAlert.set(!connected);

    inputs.connected = connected;
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    inputs.odometryNTJNITimestamps = NetworkTablesJNI.now();
    inputs.yawPositionCANLatency = yaw.getTimestamp().getLatency();
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
