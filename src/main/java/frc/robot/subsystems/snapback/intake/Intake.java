package frc.robot.subsystems.snapback.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public boolean sensorTriggered() {
        return inputs.sensorValue;
    }

    private Command runAllMotors(double voltage) {
        return Commands.parallel(Commands.run(() -> io.setIntakeVoltage(voltage)),
                Commands.run(() -> io.setSerializerVoltage(voltage)),
                Commands.run(() -> io.setKickerVoltage(voltage / 2)));

    }

    public Command intake() {
        return runEnd(() -> {
            runOnce(() -> io.setActuatorValue(Value.kForward));
            runAllMotors(12).until(() -> sensorTriggered());
        }, () -> {
            runOnce(() -> io.setActuatorValue(Value.kReverse));
            runAllMotors(0);
        });
    }

    public Command outtake() {
        return runEnd(() -> runAllMotors(-12), () -> runAllMotors(0));
    }
    

}
