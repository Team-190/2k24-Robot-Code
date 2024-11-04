package frc.robot.subsystems.snapback.shooter;

import frc.robot.constants.Constants;

public class ShooterConstants {
  public static final Gains GAINS;

  static {
    switch (Constants.ROBOT) {
      case WHIPLASH:
        GAINS = new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        break;
      default:
        GAINS = new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        break;
    }
  }

  public record Gains(double kp, double ki, double kd, double ks, double kv, double ka) {}
}
