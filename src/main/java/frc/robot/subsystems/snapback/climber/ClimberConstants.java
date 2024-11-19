package frc.robot.subsystems.snapback.climber;

import frc.robot.constants.Constants;

public class ClimberConstants {
    public static final int LEFT_CLIMBER_CAN_ID;
    public static final int RIGHT_CLIMBER_CAN_ID;

    public static final int LEFT_CLIMBER_SOLENOID_FORWARD;
    public static final int LEFT_CLIMBER_SOLENOID_REVERSE;
    public static final int RIGHT_CLIMBER_SOLENOID_FORWARD;
    public static final int RIGHT_CLIMBER_SOLENOID_REVERSE;

    static {
        switch (Constants.ROBOT) {
            case SNAPBACK:
                LEFT_CLIMBER_CAN_ID = 0;
                RIGHT_CLIMBER_CAN_ID = 1;
                
                LEFT_CLIMBER_SOLENOID_FORWARD = 0;
                LEFT_CLIMBER_SOLENOID_REVERSE = 1;
                RIGHT_CLIMBER_SOLENOID_FORWARD = 2;
                RIGHT_CLIMBER_SOLENOID_REVERSE = 3;
                break;
        
            default:
                LEFT_CLIMBER_CAN_ID = 0;
                RIGHT_CLIMBER_CAN_ID = 1;

                LEFT_CLIMBER_SOLENOID_FORWARD = 0;
                LEFT_CLIMBER_SOLENOID_REVERSE = 1;
                RIGHT_CLIMBER_SOLENOID_FORWARD = 2;
                RIGHT_CLIMBER_SOLENOID_REVERSE = 3;
                break;
        }
    }

}
