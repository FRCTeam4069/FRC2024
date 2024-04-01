package frc.robot.constants;

import frc.robot.commands.drivebase.test.straightLineTest;

public final class DeviceIDs {
    public static final byte INTAKE_ARTICULATE = 7; 
    public static final byte INTAKE_FEED = 10; 

    public static final byte FEEDER = 12;

    public static final byte SHOOTER_ARTICULATE_LEFT = 5;
    public static final byte SHOOTER_ARTICULATE_RIGHT = 6;

    public static final byte SHOOTER_FLYWHEEL_LEFT = 8;
    public static final byte SHOOTER_FLYWHEEL_RIGHT = 9;

    public static final byte DRIVE_FL = 4;
    public static final byte DRIVE_FR = 3;
    public static final byte DRIVE_BL = 2;
    public static final byte DRIVE_BR = 1;
    
    public static final byte STEER_FL = 24;
    public static final byte STEER_FR = 23;
    public static final byte STEER_BL = 22;
    public static final byte STEER_BR = 21;

    public static final byte ENCODER_FL = 34;
    public static final byte ENCODER_FR = 33;
    public static final byte ENCODER_BL = 32;
    public static final byte ENCODER_BR = 31;

    public static final byte CLIMBER = 40;

    // PWM Channels not CAN
    public static final byte AMPARM_LEFT = 8;
    public static final byte AMPARM_RIGHT = 9;

    // From Ahmed's Constants file
    public static final byte PHOTO_ELECTRIC_SENSOR_PORT = 2;
}
