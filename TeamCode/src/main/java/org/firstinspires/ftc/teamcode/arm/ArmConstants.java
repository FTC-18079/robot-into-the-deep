package org.firstinspires.ftc.teamcode.arm;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmConstants {
    /**
     * SLIDES
     */

    // Motion profile constraints
    public static double MAX_VELOCITY = 182.0;
    public static double MAX_ACCELERATION = MAX_VELOCITY * 4.0;

    // PID
    public static double SLIDE_kP = 0.025;
    public static double SLIDE_kI = 0.0;
    public static double SLIDE_kD = 0.0003;
    public static double SLIDE_kF = 0.15;

    // Alignment PID
    public static double ALIGN_kP = 0.025;
    public static double ALIGN_kI = 0.0;
    public static double ALIGN_kD = 0.0;

    // Constants
    public static double SLIDE_ERROR_TOLERANCE = 15.0;
    public static double ALIGN_ERROR_TOLERANCE = 2;
    public static double SLIDE_TIMEOUT = 1400;

    // Positions
    public static double SLIDE_REST_POSITION = 5.0;
    public static double SLIDE_SAMPLE_COLLECT_POSITION = 1150.0;
    public static double SLIDE_SPECIMEN_COLLECT_POSITION = 350;
    public static double SLIDE_CHAMBER_POSITION = SLIDE_REST_POSITION;
    public static double SLIDE_BASKET_POSITION = 1300.0;

    /**
     * PIVOT
     */

    // PID
    public static double PIVOT_kP = 0.007;
    public static double PIVOT_kI = 0.0;
    public static double PIVOT_kD = 0.00005;
    public static double PIVOT_kF = 0.05;

    // Constants
    public static double PIVOT_ERROR_TOLERANCE = 8.0;
    public static double PIVOT_GEAR_RATIO = 40.0 / 26.0;
    public static double PIVOT_TIMEOUT = 1250;

    // Positions
    public static final double PIVOT_REST_TO_SCORE_OFFSET = 150.0;
    public static double PIVOT_REST_POSITION = 18.0;
    public static double PIVOT_SCORE_POSITION = 168.0;
}
