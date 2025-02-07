package org.firstinspires.ftc.teamcode.constants;

public interface IntakeConstants {
    double SLIDE_P = 1E-4;
    double WRIST_P = 5E-3;

    double WAYPOINT_RETRACT = 0;
    double WAYPOINT_EXTEND = 20000;
    double WAYPOINT_STOW = 5000;

    double SLIDE_CURRENT_LIMIT = 3.0;

    double WRIST_UP = 280.0;
    double WRIST_STOW = 210.0;

    //for quadratic regression
    double WRIST_DOWN_A = 5.33E-8;
    double WRIST_DOWN_B = 5.48E-4;
    double WRIST_DOWN_C = 169.6;
}
