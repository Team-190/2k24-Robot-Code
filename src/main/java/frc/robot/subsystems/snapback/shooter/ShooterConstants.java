package frc.robot.subsystems.snapback.shooter;

import frc.robot.Constants;

public class ShooterConstants {
  private static final Gains GAINS;

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
