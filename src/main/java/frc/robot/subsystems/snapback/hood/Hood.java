package frc.robot.subsystems.snapback.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  public Hood(HoodIO io) {
    this.io = io;
  }
}
