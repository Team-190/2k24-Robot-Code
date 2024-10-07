package frc.robot.commands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.drive.Drive;
import frc.robot.subsystems.drive.drive.DriveConstants;
import frc.robot.util.AllianceFlipUtil;

public final class AutoRoutines {
  public static final Command none() {
    return Commands.none();
  }

  public static final Command getInitialChoreoCommand(Drive drive, String trajectory) {
    PIDController xFeedback =
        new PIDController(DriveConstants.AUTO_X_KP.get(), 0.0, DriveConstants.AUTO_X_KD.get());
    PIDController yFeedback =
        new PIDController(DriveConstants.AUTO_Y_KP.get(), 0.0, DriveConstants.AUTO_Y_KD.get());
    PIDController thetaFeedback =
        new PIDController(
            DriveConstants.AUTO_THETA_KP.get(), 0.0, DriveConstants.AUTO_THETA_KD.get());
    thetaFeedback.enableContinuousInput(-Math.PI, Math.PI);
    ChoreoTrajectory choreoTrajectory = Choreo.getTrajectory(trajectory);
    return Commands.sequence(
        Commands.runOnce(
            () ->
                RobotState.resetRobotPose(
                    AllianceFlipUtil.apply(choreoTrajectory.getInitialPose()))),
        Choreo.choreoSwerveCommand(
            choreoTrajectory,
            RobotState::getRobotPose,
            xFeedback,
            yFeedback,
            thetaFeedback,
            (ChassisSpeeds speeds) -> drive.runVelocity(speeds),
            () -> AllianceFlipUtil.shouldFlip(),
            drive));
  }

  public static final Command getChoreoCommand(Drive drive, String trajectory) {
    PIDController xFeedback =
        new PIDController(DriveConstants.AUTO_X_KP.get(), 0.0, DriveConstants.AUTO_X_KD.get());
    PIDController yFeedback =
        new PIDController(DriveConstants.AUTO_Y_KP.get(), 0.0, DriveConstants.AUTO_Y_KD.get());
    PIDController thetaFeedback =
        new PIDController(
            DriveConstants.AUTO_THETA_KP.get(), 0.0, DriveConstants.AUTO_THETA_KD.get());
    thetaFeedback.enableContinuousInput(-Math.PI, Math.PI);
    ChoreoTrajectory choreoTrajectory = Choreo.getTrajectory(trajectory);
    return Choreo.choreoSwerveCommand(
        choreoTrajectory,
        RobotState::getRobotPose,
        xFeedback,
        yFeedback,
        thetaFeedback,
        (ChassisSpeeds speeds) -> drive.runVelocity(speeds),
        () -> AllianceFlipUtil.shouldFlip(),
        drive);
  }
}
