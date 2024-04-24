package frc.robot.subsystems.amp;

import org.littletonrobotics.junction.AutoLog;

public interface AmpIO {
  @AutoLog
  public static class AmpIOInputs {
    public boolean position = false;
  }

  public default void updateInputs(AmpIOInputs inputs) {}

  public default void setPosition(boolean isDeployed) {}
}
