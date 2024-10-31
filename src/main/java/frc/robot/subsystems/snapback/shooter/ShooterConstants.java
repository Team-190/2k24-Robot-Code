package frc.robot.subsystems.snapback.shooter;

public class ShooterConstants {
  static {
  }

  public record Gains(double kp, double ki, double kd, double ks, double kv, double ka) {}
}
