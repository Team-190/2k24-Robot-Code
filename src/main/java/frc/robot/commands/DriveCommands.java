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

package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.ShotCalculator;
import frc.robot.ShotCalculator.AimingParameters;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionMode;
import frc.robot.util.LoggedTunableNumber;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final LoggedTunableNumber autoAimKP = new LoggedTunableNumber("autoaim/kp");
  private static final LoggedTunableNumber autoAimKD = new LoggedTunableNumber("autoaim/kd");
  private static final LoggedTunableNumber autoAimXVelMax =
      new LoggedTunableNumber("autoaim/xvelmax", 3.0);
  private static final LoggedTunableNumber autoAimXVelMin =
      new LoggedTunableNumber("autoaim/xvelmin", 0.5);
  private static final LoggedTunableNumber autoAimXVelRange =
      new LoggedTunableNumber("autoaim/xvelrange", 0.5);
  private static final LoggedTunableNumber autoAimFieldVelocityDeadband =
      new LoggedTunableNumber("autoaim/fieldvelocitydeadband", 0.5);

  static {
    switch (Constants.ROBOT) {
      case SNAPBACK:
        autoAimKP.initDefault(6.0);
        autoAimKD.initDefault(0.002);
        break;
      case ROBOT_2K24_TEST:
        autoAimKP.initDefault(6.0);
        autoAimKD.initDefault(0.002);
        break;
      case ROBOT_SIM:
        autoAimKP.initDefault(7);
        autoAimKD.initDefault(0.125);
      default:
        break;
    }
  }

  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static final Command joystickDrive(
      Drive drive,
      Vision aprilTagVision,
      Vision noteVision,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier aprilTagTracking,
      BooleanSupplier noteTracking) {

    @SuppressWarnings({"resource"})
    PIDController aimController =
        new PIDController(autoAimKP.get(), 0, autoAimKD.get(), Constants.LOOP_PERIOD_SECS);
    aimController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          if (aprilTagTracking.getAsBoolean()) {
            linearMagnitude =
                Math.min(linearMagnitude, 0.33); // change this to smaller for shoot on the move.
          }

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();
          //
          // Configure PID
          aimController.setD(autoAimKD.get());
          aimController.setP(autoAimKP.get());

          // Get robot relative vel
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          Optional<Rotation2d> targetGyroAngle = Optional.empty();
          Rotation2d measuredGyroAngle = drive.getRotation();
          double feedForwardRadialVelocity = 0.0;

          double robotRelativeXVel = linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec();
          double robotRelativeYVel = linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec();

          if (noteTracking.getAsBoolean()) {
            targetGyroAngle = noteVision.getTargetGyroAngle();
          } else if (aprilTagVision.getRobotPose().isPresent()) {
            Pose2d visionPose = aprilTagVision.getRobotPose().get();
            measuredGyroAngle = visionPose.getRotation();
            Translation2d deadbandFieldRelativeVelocity =
                (drive.getFieldRelativeVelocity().getNorm() < autoAimFieldVelocityDeadband.get())
                    ? new Translation2d(0, 0)
                    : drive.getFieldRelativeVelocity();
            AimingParameters calculatedAim =
                ShotCalculator.poseCalculation(
                    visionPose.getTranslation(), deadbandFieldRelativeVelocity);
            targetGyroAngle = Optional.of(calculatedAim.robotAngle());
            feedForwardRadialVelocity = calculatedAim.radialVelocity();
          }
          ChassisSpeeds chassisSpeeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  robotRelativeXVel,
                  robotRelativeYVel,
                  (aprilTagTracking.getAsBoolean() || noteTracking.getAsBoolean())
                          && targetGyroAngle.isPresent()
                      ? feedForwardRadialVelocity
                          + aimController.calculate(
                              measuredGyroAngle.getRadians(), targetGyroAngle.get().getRadians())
                      : omega * drive.getMaxAngularSpeedRadPerSec(),
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation());
          if (noteTracking.getAsBoolean()) {
            chassisSpeeds.vyMetersPerSecond = 0;
          }

          // Convert to field relative speeds & send command
          drive.runVelocity(chassisSpeeds);
        },
        drive);
  }

  public static final Command XLock(Drive drive) {
    return Commands.runOnce(drive::stopWithX);
  }

  public static final Command resetHeading(Drive drive) {
    return Commands.runOnce(
            () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
            drive)
        .ignoringDisable(true);
  }

  public static final Command aimTowardsTarget(
      Drive drive, Vision aprilTagVision, VisionMode targetType) {
    @SuppressWarnings({"resource"})
    PIDController aimController =
        new PIDController(autoAimKP.get(), 0, autoAimKD.get(), Constants.LOOP_PERIOD_SECS);
    aimController.enableContinuousInput(-Math.PI, Math.PI);
    aimController.setTolerance(0.017);

    return Commands.run(
            () -> {
              aimController.setD(autoAimKD.get());
              aimController.setP(autoAimKP.get());

              // Get robot relative vel
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              Optional<Rotation2d> targetGyroAngle = Optional.empty();
              Rotation2d measuredGyroAngle = drive.getRotation();
              double feedForwardRadialVelocity = 0.0;

              if (aprilTagVision.getRobotPose().isPresent()) {
                Pose2d visionPose = aprilTagVision.getRobotPose().get();
                measuredGyroAngle = visionPose.getRotation();
                Translation2d deadbandFieldRelativeVelocity =
                    (drive.getFieldRelativeVelocity().getNorm()
                            < autoAimFieldVelocityDeadband.get())
                        ? new Translation2d(0, 0)
                        : drive.getFieldRelativeVelocity();
                AimingParameters calculatedAim =
                    ShotCalculator.poseCalculation(
                        visionPose.getTranslation(), deadbandFieldRelativeVelocity);
                targetGyroAngle = Optional.of(calculatedAim.robotAngle());
                feedForwardRadialVelocity = calculatedAim.radialVelocity();
              }
              ChassisSpeeds chassisSpeeds =
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      0,
                      0,
                      feedForwardRadialVelocity
                          + (targetGyroAngle.isPresent() ? aimController.calculate(
                              measuredGyroAngle.getRadians(), targetGyroAngle.get().getRadians()) : 0),
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation());

              // Convert to field relative speeds & send command
              drive.runVelocity(chassisSpeeds);
            },
            drive)
        .until(() -> aimController.atSetpoint())
        .finallyDo(
            () -> {
              drive.stop();
              aimController.reset();
            });
  }

  public static final Command moveTowardsTarget(
      Drive drive, Vision vision, double blueXCoord, VisionMode targetType) {

    @SuppressWarnings({"resource"})
    PIDController aimController =
        new PIDController(autoAimKP.get(), 0, autoAimKD.get(), Constants.LOOP_PERIOD_SECS);
    aimController.enableContinuousInput(-Math.PI, Math.PI);

    DoubleSupplier targetXCoord =
        () -> {
          boolean isRed =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get().equals(Alliance.Red);
          return isRed ? FieldConstants.fieldLength - blueXCoord : blueXCoord;
        };

    return Commands.run(
            () -> {
              // Configure PID
              aimController.setD(autoAimKD.get());
              aimController.setP(autoAimKP.get());

              // Convert to field relative speeds & send command
              Optional<Rotation2d> targetGyroAngle = vision.getTargetGyroAngle();
              double distanceT =
                  MathUtil.clamp(
                      Math.abs(drive.getPose().getX() - targetXCoord.getAsDouble())
                          / autoAimXVelRange.get(),
                      0.0,
                      1.0);
              double speed =
                  MathUtil.interpolate(autoAimXVelMin.get(), autoAimXVelMax.get(), distanceT);
              drive.runVelocity(
                  new ChassisSpeeds(
                      targetType.equals(VisionMode.AprilTags) ? speed : -speed,
                      0,
                      targetGyroAngle.isEmpty()
                          ? 0.0
                          : aimController.calculate(
                              drive.getRotation().getRadians(),
                              targetGyroAngle.get().getRadians())));
            },
            drive)
        .until(
            () -> {
              boolean endAboveTargetXCoord;
              boolean isRed =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get().equals(Alliance.Red);
              if (isRed) {
                endAboveTargetXCoord = targetType.equals(VisionMode.AprilTags);
              } else {
                endAboveTargetXCoord = targetType.equals(VisionMode.Notes);
              }
              if (endAboveTargetXCoord) {
                return drive.getPose().getX() > targetXCoord.getAsDouble();
              } else {
                return drive.getPose().getX() < targetXCoord.getAsDouble();
              }
            })
        .finallyDo(() -> drive.stop());
  }

  public static final Command runSysIdQuasistatic(Drive drive, Direction direction) {
    return new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null, (state) -> Logger.recordOutput("SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> drive.runCharacterizationVolts(volts.in(Volts)), null, drive))
        .quasistatic(direction);
  }

  public static final Command runSysIdDynamic(Drive drive, Direction direction) {
    return new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null, (state) -> Logger.recordOutput("SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> drive.runCharacterizationVolts(volts.in(Volts)), null, drive))
        .dynamic(direction);
  }
}
