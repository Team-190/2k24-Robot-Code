package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class GrantFieldRelativeDrive extends CommandBase {
  private final Drive drive;

  private final DoubleSupplier m_throttleSupplier;
  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private final DoubleSupplier m_rotationSupplier;

  private static final double MAX_LINEAR_SPEED = Units.feetToMeters(17.5);
  private static final double TRACK_WIDTH_X = Units.inchesToMeters(22.75);
  private static final double TRACK_WIDTH_Y = Units.inchesToMeters(22.75);
  private static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  private double m_lastHeading = 0;

  private boolean m_wasRotating = false;

  public GrantFieldRelativeDrive(
      Drive drivetrainSubsystem,
      DoubleSupplier throttleSupplier,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier) {
    this.drive = drivetrainSubsystem;
    this.m_throttleSupplier = throttleSupplier;
    this.m_translationXSupplier = translationXSupplier;
    this.m_translationYSupplier = translationYSupplier;
    this.m_rotationSupplier = rotationSupplier;

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void execute() {
    double heading =
        Math.atan2(-m_translationXSupplier.getAsDouble(), m_translationYSupplier.getAsDouble());
    heading -= drive.getRotation().getRadians();
    double rotation = m_rotationSupplier.getAsDouble();
    double throttle = m_throttleSupplier.getAsDouble();
    boolean pointing =
        Math.abs(m_translationXSupplier.getAsDouble()) > 0.25
            || Math.abs(m_translationYSupplier.getAsDouble()) > 0.25;

    if (!pointing) {
      heading = m_lastHeading;
    } else {
      m_lastHeading = heading;
    }

    if (Math.abs(throttle) < 0.01 && Math.abs(rotation) < 0.001) {
      if (pointing) {
        m_wasRotating = false;
        throttle = 0.01;
      } else if (m_wasRotating) {
        rotation = 0.001;
      } else {
        throttle = 0.01;
      }
    } else {
      m_wasRotating = Math.abs(rotation) >= 0.001;
    }

    double vx = throttle * Units.feetToMeters(17.5) * Math.cos(heading);
    double vy = throttle * Units.feetToMeters(17.5) * Math.sin(heading);

    // vx =
    //     MathUtil.clamp(vx, Math.copySign(0.001, vx), Math.copySign(controller.config.maxSpeed,
    // vx));
    // vy =
    //     MathUtil.clamp(vy, Math.copySign(0.001, vy), Math.copySign(controller.config.maxSpeed,
    // vy));

    rotation *= MAX_ANGULAR_SPEED;

    // swerve.drive(
    //         ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy,
    //                 rotation,
    //                 swerve.getHeading()
    //         )
    // );

    drive.runVelocity(new ChassisSpeeds(vx, vy, rotation));
  }

  @Override
  public void end(boolean interrupted) {
    // swerve.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
}
