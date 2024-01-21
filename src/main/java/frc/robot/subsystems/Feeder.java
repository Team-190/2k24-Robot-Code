package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Feeder extends SubsystemBase {
  private final TalonFX feeder = new TalonFX(53);

  private final LoggedDashboardNumber feederSpeed = new LoggedDashboardNumber("feeder speed");

  public Feeder() {
    var feederConfig = new TalonFXConfiguration();
    feederConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    feederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    feeder.getConfigurator().apply(feederConfig);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("feeder", feeder.getVelocity().getValueAsDouble());
  }

  public Command runFeeder() {
    return startEnd(
        () -> {
          feeder.setControl(new VoltageOut(feederSpeed.get()));
        },
        () -> {
          feeder.stopMotor();
        });
  }
}
