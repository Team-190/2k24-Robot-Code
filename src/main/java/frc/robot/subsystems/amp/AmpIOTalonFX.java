package frc.robot.subsystems.amp;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;

public class AmpIOTalonFX implements AmpIO {
  private final Solenoid ampSolenoid;

  public AmpIOTalonFX() {
    switch (Constants.ROBOT) {
      case SNAPBACK:
        ampSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 7);
        break;
      case ROBOT_2K24_TEST:
        ampSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 7);
        break;
      default:
        throw new RuntimeException("Invalid robot");
    }
  }

  @Override
  public void updateInputs(AmpIOInputs inputs) {
    inputs.position = ampSolenoid.get();
  }

  @Override
  public void setPosition(boolean isDeployed) {
    ampSolenoid.set(isDeployed);
  }
}
