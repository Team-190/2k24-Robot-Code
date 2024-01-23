package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Shooter extends SubsystemBase {
  private final TalonFX motor1 = new TalonFX(52);
  private final TalonFX motor2 = new TalonFX(51);

  private final LoggedDashboardNumber speed = new LoggedDashboardNumber("speed");
  private final LoggedDashboardNumber p = new LoggedDashboardNumber("p", 0.1);
  private final LoggedDashboardNumber i = new LoggedDashboardNumber("i", 0);
  private final LoggedDashboardNumber d = new LoggedDashboardNumber("d", 0.001);
  private final LoggedDashboardNumber v = new LoggedDashboardNumber("v", 0.115);

  private final VelocityVoltage velocity = new VelocityVoltage(0);

  private final Slot0Configs config = new Slot0Configs();

  public Shooter() {
    var drive1Config = new TalonFXConfiguration();
    drive1Config.CurrentLimits.StatorCurrentLimit = 40.0;
    drive1Config.CurrentLimits.StatorCurrentLimitEnable = true;
    drive1Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    var drive2Config = new TalonFXConfiguration();
    drive2Config.CurrentLimits.StatorCurrentLimit = 40.0;
    drive2Config.CurrentLimits.StatorCurrentLimitEnable = true;
    drive2Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motor1.getConfigurator().apply(drive1Config);
    motor2.getConfigurator().apply(drive2Config);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("motor 1", motor1.getVelocity().getValueAsDouble() * 2.83333333333 * 60);
    Logger.recordOutput("motor 2", motor2.getVelocity().getValueAsDouble() * 2.83333333333 * 60);

    config.kV = v.get();
    config.kP = p.get();
    config.kI = i.get();
    config.kD = d.get();

    motor1.getConfigurator().apply(config);
    motor2.getConfigurator().apply(config);
  }

  public Command runBoth() {
    return startEnd(
        () -> {
          motor1.setControl(velocity.withVelocity(speed.get() / 60 / 2.83333333333).withSlot(0));
          motor2.setControl(velocity.withVelocity(speed.get() / 60 / 2.83333333333).withSlot(0));
        },
        () -> {
          motor1.stopMotor();
          motor2.stopMotor();
        });
  }
}
