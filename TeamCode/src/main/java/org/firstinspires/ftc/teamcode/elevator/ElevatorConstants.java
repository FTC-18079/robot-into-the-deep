package org.firstinspires.ftc.teamcode.elevator;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ElevatorConstants {
    // Elevator positions
    public static double LIFT_POS_REST = 0.0;

    public static double LIFT_POS_LOW_BASKET = 950.0;
    public static double LIFT_POS_HIGH_BASKET = 2660.0;

    public static double LIFT_POS_LOW_CHAMBER = 400.0;
    public static double LIFT_POS_HIGH_CHAMBER = 1660.0;
    public static double LIFT_POS_LOW_CHAMBER_SCORE = 125.0;
    public static double LIFT_POS_HIGH_CHAMBER_SCORE = 1450.0;

    // PID constants
    public static double kP = 1.0;
    public static double kI = 0.0;
    public static double kD = 0.0;

    // Feedforward constants
    public static double kS = 0.0;
    public static double kV = 1.0;
    public static double kA = 0.0;

    // CLaw positions
    public static double CLAW_OPEN_POS = 1.0;
    public static double CLAW_GRAB_POS = 0.55;

    public static double BUCKET_REST_POS = 0.0;
    public static double BUCKET_SCORE_POS = 0.0;
}
