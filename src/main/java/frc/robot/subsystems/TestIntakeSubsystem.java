package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class TestIntakeSubsystem extends SubsystemBase {
  private final TalonFX motor1 = new TalonFX(53);

  private final LoggedDashboardNumber speed = new LoggedDashboardNumber("speed");
 
  private final VoltageOut voltageOut = new VoltageOut(0);

  public TestIntakeSubsystem() {
    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    motor1.getConfigurator().apply(driveConfig);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("motor 1", motor1.getVelocity().getValueAsDouble() * 2.2037 * 60);
  }

  public Command runMotor1() {
    return startEnd(
        () -> {
          motor1.setControl(voltageOut.withOutput(speed.get()));
        },
        () -> {
          motor1.stopMotor();
        });
  }
}

