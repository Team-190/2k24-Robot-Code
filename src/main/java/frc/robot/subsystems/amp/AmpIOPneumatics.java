package frc.robot.subsystems.amp;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

public class AmpIOPneumatics implements AmpIO {
  private final DoubleSolenoid ampDoubleSolenoid;

  public AmpIOPneumatics() {
    switch (Constants.ROBOT) {
      case ROBOT_2K24_C:
        ampDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);
        break;
      case ROBOT_2K24_P:
        ampDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);
        break;
      case ROBOT_2K24_TEST:
        ampDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);
        break;
      default:
        throw new RuntimeException("Invalid robot");
    }
  }

  @Override
  public void updateInputs(AmpIOInputs inputs) {
    inputs.position = ampDoubleSolenoid.get();
  }

  @Override
  public void setPosition(DoubleSolenoid.Value position) {
    ampDoubleSolenoid.set(position);
  }
}
