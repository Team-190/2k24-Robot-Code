package frc.robot.subsystems.amp;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.littletonrobotics.junction.AutoLog;

public interface AmpIO {
  @AutoLog
  public static class AmpIOInputs {
    public DoubleSolenoid.Value position = DoubleSolenoid.Value.kReverse;
  }

  public default void updateInputs(AmpIOInputs inputs) {}

  public default void setPosition(DoubleSolenoid.Value position) {}
}
