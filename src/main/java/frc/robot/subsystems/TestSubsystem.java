package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class TestSubsystem extends SubsystemBase {
  private final TalonFX motor1 = new TalonFX(52);
  private final TalonFX motor2 = new TalonFX(51);

  private final LoggedDashboardNumber speed = new LoggedDashboardNumber("speed");
  private final LoggedDashboardNumber p = new LoggedDashboardNumber("p", 0.1);
  private final LoggedDashboardNumber i = new LoggedDashboardNumber("i", 0);
  private final LoggedDashboardNumber d = new LoggedDashboardNumber("d", 0.001);
  private final LoggedDashboardNumber v = new LoggedDashboardNumber("v", 0.115);

  private final VelocityVoltage velocity = new VelocityVoltage(0);

  private final Slot0Configs config = new Slot0Configs();

  public TestSubsystem() {
    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    motor1.getConfigurator().apply(driveConfig);
    motor2.getConfigurator().apply(driveConfig);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("motor 1", motor1.getVelocity().getValueAsDouble() * 2.2037 * 60);
    Logger.recordOutput("motor 2", motor2.getVelocity().getValueAsDouble() * 2.2037 * 60);

    config.kV = v.get();
    config.kP = p.get();
    config.kI = i.get();
    config.kD = d.get();

    motor1.getConfigurator().apply(config);
    motor2.getConfigurator().apply(config);
  }

  public Command runMotor1() {
    return startEnd(
        () -> {
          motor1.setControl(velocity.withVelocity(speed.get() / 60 / 2.2037).withSlot(0));
        },
        () -> {
          motor1.stopMotor();
        });
  }

  public Command runMotor2() {
    return startEnd(
        () -> {
          motor2.setControl(velocity.withVelocity(speed.get() / 60 / 2.2037).withSlot(0));
        },
        () -> {
          motor2.stopMotor();
        });
  }

  public Command runBoth() {
    return startEnd(
        () -> {
          motor1.setControl(velocity.withVelocity(speed.get() / 60 / 2.2037).withSlot(0));
          motor2.setControl(velocity.withVelocity(speed.get() / 60 / 2.2037).withSlot(0));
        },
        () -> {
          motor1.stopMotor();
          motor2.stopMotor();
        });
  }
}
