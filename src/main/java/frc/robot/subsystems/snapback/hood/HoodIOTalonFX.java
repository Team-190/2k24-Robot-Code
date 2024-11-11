package frc.robot.subsystems.snapback.hood;

public class HoodIOTalonFX implements HoodIO {
    private final TalonFX hoodMotor;
    public StatusSignal<Angle> positionRadians;
    public StatusSignal<Velocity> velocityRadiansPerSecond;
    public StatusSignal<Current> currentAmps;
    public StatusSignal<Temprature> tempratureCelsius;

    private final Alert disconnectedAlert = new Alert("Hood Talon is disconnected, check CAN bus.", AlertType.ERROR);

    private VoltageOut voltageControl;
    private MotionMagicVoltage positionControl = new MotionMagicVoltage(0);

    public HoodIOTalonFX() {
        hoodMotor = new TalonFX(HoodConstants.HOOD_MOTOR_CAN_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 60.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = HoodConstants.INTAKE_GEAR_REDUCTION;
        config.Slot0.kP = HoodConstants.GAINS.kp();
        config.Slot0.kI = HoodConstants.GAINS.ki();
        config.Slot0.kD = HoodConstants.GAINS.kd();
        config.Slot0.kS = HoodConstants.GAINS.ks();
        config.Slot0.kV = HoodConstants.GAINS.kv();
        config.Slot0.kA = HoodConstants.GAINS.ka();
        hoodMotor.getConfigurator.apply(config);

        positionRadians = hoodMotor.getPosition();
        velocityRadiansPerSecond = hoodMotor.getVelocity();
        currentAmps = hoodMotor.getCurrent();
        tempratureCelsius = hoodMotor.getTemprature();

        BaseStatusSignal.setUpdateFrequencyAll(50, positionRadians, velocityRadiansPerSecond, currentAmps,
                tempratureCelsius);
        hoodMotor.optimizeBusUtilization();
    }

    public void updateInputs(HoodIOInputs inputs) {
        boolean isConnected = hoodMotor.refreshAll(
                positionRadians, velocityRadiansPerSecond, currentAmps, tempratureCelsius).isOK();
        disconnectedAlert.set(!isConnected);

        inputs.position = positionRadians.getValueAsDouble();
        inputs.velocity = velocityRadiansPerSecond.getValueAsDouble();
        inputs.current = currentAmps.getValueAsDouble();
        inputs.temprature = tempratureCelsius.getValueAsDouble();
    }

    public void setVoltage(double voltage) {
        hoodMotor.setControl(voltageControl.withOutput(volts).withEnableFOC(true));
    }

    public void setPosition(double position) { // position in rotations!!!
        hoodMotor.setControl(positionControl.withPosition(position).withEnableFOC(true));
    }

}
