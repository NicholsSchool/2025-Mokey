package org.firstinspires.ftc.teamcode.constants;

public interface IntakeConstants {
    double SLIDE_P = 0.0001;

    double WAYPOINT_RETRACT = 0;
    //double WAYPOINT_EXTEND = -24000;
    double WAYPOINT_EXTEND = -10000;

    double INTAKE_SPEED = 0.5;

    // Colour sensor constants for red sample
    int CSENS_RED_R = 1000;
    int CSENS_RED_G = 20;
    int CSENS_RED_B = 20;

    //Colour sensor constants for blue sample
    int CSENS_BLUE_R = 20;
    int CSENS_BLUE_G = 20;
    int CSENS_BLUE_B = 1000;

    double INTAKE_WRIST_FRONT_IN = 280.0;
    double INTAKE_WRIST_FRONT_OUT = 210.0;
    double INTAKE_LIMIT = -100;
}
