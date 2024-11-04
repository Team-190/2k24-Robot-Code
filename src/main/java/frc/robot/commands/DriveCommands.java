package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotState;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.drive.DriveConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public final class DriveCommands {
    private static PIDController aimController =
        new PIDController(
            DriveConstants.AUTO_THETA_KP,
            0,
            DriveConstants.AUTO_THETA_KD,
            Constants.LOOP_PERIOD_SECONDS);

    static {
      aimController.setTolerance(Units.degreesToRadians(1.0));
      aimController.enableContinuousInput(-Math.PI, Math.PI);
    }
  
    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    public static final Command joystickDrive(
        Drive drive,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        DoubleSupplier omegaSupplier,
        BooleanSupplier speakerAim,
        BooleanSupplier ampAim,
        BooleanSupplier feedAim) {
      return Commands.run(
          () -> {
            aimController.enableContinuousInput(-Math.PI, Math.PI);
  
            // Apply deadband
            double linearMagnitude =
                MathUtil.applyDeadband(
                    Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
                    DriveConstants.DRIVER_DEADBAND);
            Rotation2d linearDirection =
                new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
            double omega =
                MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DriveConstants.DRIVER_DEADBAND);
  
            // Square values
            linearMagnitude = linearMagnitude * linearMagnitude;
  
            // Calcaulate new linear velocity
            Translation2d linearVelocity =
                new Pose2d(new Translation2d(), linearDirection)
                    .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                    .getTranslation();
  
            // Get robot relative vel
            boolean isFlipped =
                DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red;
  
            double robotRelativeXVel = linearVelocity.getX() * DriveConstants.MAX_LINEAR_VELOCITY;
            double robotRelativeYVel = linearVelocity.getY() * DriveConstants.MAX_LINEAR_VELOCITY;
  
            double angular = 0.0;
  
            if (speakerAim.getAsBoolean()) {
              angular =
                  RobotState.getControlData().speakerRadialVelocity()
                      + (aimController.calculate(
                          RobotState.getRobotPose().getRotation().getRadians(),
                          RobotState.getControlData().speakerRobotAngle().getRadians()));
            } else if (ampAim.getAsBoolean()) {
              angular =
                  RobotState.getControlData().speakerRadialVelocity()
                      + (aimController.calculate(
                          RobotState.getRobotPose().getRotation().getRadians(),
                          Rotation2d.fromDegrees(90.0).getRadians()));
            } else if (feedAim.getAsBoolean()) {
              angular =
                  RobotState.getControlData().speakerRadialVelocity()
                      + (aimController.calculate(
                          RobotState.getRobotPose().getRotation().getRadians(),
                          Rotation2d.fromDegrees(-35.5).getRadians()));
            } else {
              angular = omega * DriveConstants.MAX_ANGULAR_VELOCITY;
            }
  
            ChassisSpeeds chassisSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    robotRelativeXVel,
                    robotRelativeYVel,
                    angular,
                    isFlipped
                        ? RobotState.getRobotPose().getRotation().plus(new Rotation2d(Math.PI))
                        : RobotState.getRobotPose().getRotation());
  
            // Convert to field relative speeds & send command
            drive.runVelocity(chassisSpeeds);
          },
          drive);
    }
  
    public static final Command aimTowardSpeaker(Drive drive) {
      return Commands.run(
              () -> {
                boolean isFlipped =
                    DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red;
  
                ChassisSpeeds chassisSpeeds =
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        0,
                        0,
                        RobotState.getControlData().speakerRadialVelocity()
                            + (aimController.calculate(
                                RobotState.getRobotPose().getRotation().getRadians(),
                                RobotState.getControlData().speakerRobotAngle().getRadians())),
                        isFlipped
                            ? RobotState.getRobotPose().getRotation().plus(new Rotation2d(Math.PI))
                            : RobotState.getRobotPose().getRotation());
  
                // Convert to field relative speeds & send command
                drive.runVelocity(chassisSpeeds);
              },
              drive)
          .finallyDo(
              () -> {
                drive.stop();
              });
    }
  
    public static final Command stop(Drive drive) {
      return Commands.run(() -> drive.stopWithX());
    }
  
    public static final Command runSysIdQuasistatic(Drive drive, Direction direction) {
      return new SysIdRoutine(
              new SysIdRoutine.Config(
                  null,
                  null,
                  null,
                  (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
              new SysIdRoutine.Mechanism(
                  (volts) -> drive.runCharacterization(volts.in(Volts)), null, drive))
          .quasistatic(direction);
    }
  
    public static final Command runSysIdDynamic(Drive drive, Direction direction) {
      return new SysIdRoutine(
              new SysIdRoutine.Config(
                  null,
                  null,
                  null,
                  (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
              new SysIdRoutine.Mechanism(
                  (volts) -> drive.runCharacterization(volts.in(Volts)), null, drive))
          .dynamic(direction);
    }
  
    public static final boolean atAimSetpoint() {
      return aimController.atSetpoint();
    }
}