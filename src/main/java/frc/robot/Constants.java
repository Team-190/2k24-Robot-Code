package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  public static final boolean TUNING_MODE = false;
  public static final double LOOP_PERIOD_SECONDS = 0.02;
  public static final RobotType ROBOT = RobotType.WHIPLASH;

  public static Mode getMode() {
    switch (ROBOT) {
      case WHIPLASH:
      case SNAPBACK:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case SNAPBACK_SIM:
      case WHIPLASH_SIM:
        return Mode.SIM;

      default:
        return Mode.REAL;
    }
  }

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static enum RobotType {
    SNAPBACK,
    WHIPLASH,
    SNAPBACK_SIM,
    WHIPLASH_SIM
  }

  public static void main(String... args) {
    if (ROBOT == RobotType.SNAPBACK_SIM) {
      System.err.println("Cannot deploy, invalid mode selected: " + ROBOT.toString());
      System.exit(1);
    }
  }
}
