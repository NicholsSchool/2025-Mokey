package org.firstinspires.ftc.teamcode.constants;

/**
 * Constants for the Drivetrain
 */
public interface DrivetrainConstants {

    /* The threshold at which to stop driving in driveToPose */
    double DRIVE_SETPOINT_THRESHOLD = 1.0;

    /* The threshold at which to stop auto-aligning in driveToPose */
    double TURN_SETPOINT_THRESHOLD = 3.0;

    double DRIVE_TO_POS_PROPORTIONAL = 0.25;

    /** The Maximum Speed of the Driving Profile */
    double DRIVE_PROFILE_SPEED = 10.0;

    /** The Maximum Speed of the Turning Profile */
    double TURN_PROFILE_SPEED = 5.0;

    /** The Maximum Output Value Magnitude of the Turning Profile */
    double TURN_PROFILE_MAX = 0.5;

    /** Virtual Low Gear Max Speed */
    double VIRTUAL_LOW_GEAR = 0.75;

    /** Virtual High Gear Max Speed */
    double VIRTUAL_HIGH_GEAR = 1;

    /** Auto Align Proportional Constant */
    double AUTO_ALIGN_P = 0.45;

    /** Auto Align allowed error in radians (.5 degrees) */
    double AUTO_ALIGN_ERROR = 0.00872664625997;

    /** Left Drive Wheel Angle Offset (30 degrees) */
    double LEFT_DRIVE_OFFSET = Math.PI / 6.0;

    /** Right Drive Wheel Angle Offset (150 degrees) */
    double RIGHT_DRIVE_OFFSET = 5.0 * Math.PI / 6.0;

    /** Back Drive Wheel Angle Offset (270 degrees) */
    double BACK_DRIVE_OFFSET = 1.5 * Math.PI;

    /** Drive Motor Proportional Gain */
    double DRIVE_P = 16.0;

    /** Drive Motor Integral Gain */
    double DRIVE_I = 6.0;
}