package org.firstinspires.ftc.teamcode.elevator;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ElevatorConstants {
    public static double MAX_DELTAV = 700.0;

    // Elevator positions
    public static double LIFT_POS_REST = 0.0;

    public static double LIFT_POS_LOW_BASKET = 750.0;
    public static double LIFT_POS_HIGH_BASKET = 1900.0;

    public static double LIFT_POS_HIGH_CHAMBER = 1075.0;
    public static double LIFT_POS_HIGH_CHAMBER_SCORE = 750.0;

    public static double POSITION_TOLERANCE = 10.0;

    // PID constants
    public static double kP = 5.0;
    public static double kI = 0.0;
    public static double kD = 0.02;
    public static double kF = 0.012;

    // CLaw positions
    public static double CLAW_OPEN_POS = 1.0;
    public static double CLAW_GRAB_POS = 0.55;

    // Door positions
    public static double DOOR_CLOSE_POS = 0.0;
    public static double DOOR_OPEN_POS = 0.6;

    // Bucket positions
    public static double BUCKET_REST_POS = 0.0;
    public static double BUCKET_SCORE_POS = 1.0;
}
