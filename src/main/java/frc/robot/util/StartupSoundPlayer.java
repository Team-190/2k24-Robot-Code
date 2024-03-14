package frc.robot.util;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.Arrays;
import java.util.Random;

public class StartupSoundPlayer {
  private static final int NUM_OF_SOUNDS = 1;

  private static Orchestra[] parts =
      new Orchestra[] {new Orchestra(), new Orchestra(), new Orchestra()};

  private static int soundNum = 1;

  public static void chooseSound() {
    Arrays.fill(parts, new Orchestra());

    Random random = new Random();
    soundNum = random.nextInt(1, NUM_OF_SOUNDS + 1);
    for (int i = 0; i < parts.length; i++) {
      // parts[i].loadMusic(getSoundFilepath(i + 1));
      parts[i].loadMusic("1 - 1.chrp");
    }
  }

  public static String getSoundFilepath(int part) {
    return /*"startupsounds\\" + */ soundNum + " - " + part + ".chrp";
  }

  public static void addInstrument(TalonFX instrument, int part) {
    parts[part - 1].addInstrument(instrument);
  }

  public static void playStartupSound() {
    for (Orchestra part : parts) {
      part.play();
    }
  }
}
