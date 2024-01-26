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

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean TUNING_MODE = true;
  public static final double LOOP_PERIOD_SECS = 0.02;
  public static final RobotType ROBOT = RobotType.ROBOT_SIM;
  public static final Mode MODE;

  static {
    switch (ROBOT) {
      case ROBOT_2K24_C:
      case ROBOT_2K24_P:
        MODE = RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
        break;

      case ROBOT_SIM:
        MODE = Mode.SIM;
        break;

      default:
        MODE = Mode.REAL;
        break;
    }
  }

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static enum RobotType {
    ROBOT_2K24_C,
    ROBOT_2K24_P,
    ROBOT_2K23_EMBER,
    ROBOT_SIM
  }

  /** Checks whether the robot the correct mode is selected when deploying. */
  public static void main(String... args) {
    if (ROBOT == RobotType.ROBOT_SIM) {
      System.err.println("Cannot deploy, invalid mode selected: " + ROBOT.toString());
      System.exit(1);
    }
  }
}
