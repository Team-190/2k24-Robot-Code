package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class TestSubsystem extends SubsystemBase {
  private final TalonFX motor1 = new TalonFX(52);
  private final TalonFX motor2 = new TalonFX(51);

  private final LoggedDashboardNumber motor1Volts = new LoggedDashboardNumber("Motor1 speed");
  private final LoggedDashboardNumber motor2Volts = new LoggedDashboardNumber("Motor2 speed");

  public TestSubsystem() {
    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motor1.getConfigurator().apply(driveConfig);
    motor2.getConfigurator().apply(driveConfig);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("RPS", motor1.getVelocity().getValue());
  }

  public Command runMotor1() {
    return startEnd(
        () -> {
          motor1.setControl(new VoltageOut(motor1Volts.get()));
        },
        () -> {
          motor1.stopMotor();
        });
  }

  public Command runMotor2() {
    return startEnd(
        () -> {
          motor2.setControl(new VoltageOut(motor2Volts.get()));
        },
        () -> {
          motor2.stopMotor();
        });
  }

  public Command runBoth() {
    return startEnd(
        () -> {
          motor1.setControl(new VoltageOut(motor1Volts.get()));
          motor2.setControl(new VoltageOut(motor2Volts.get()));
        },
        () -> {
          motor1.stopMotor();
          motor2.stopMotor();
        });
  }
}
