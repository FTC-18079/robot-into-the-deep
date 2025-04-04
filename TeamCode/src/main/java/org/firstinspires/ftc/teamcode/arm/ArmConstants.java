package org.firstinspires.ftc.teamcode.arm;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmConstants {

    /**
     * SLIDES
     */

    // PID
    public static double SLIDE_kP = 0.022;
    public static double SLIDE_kI = 0.0;
    public static double SLIDE_kD = 0.0003;
    public static double SLIDE_kF = 0.1;

    // Alignment PID
    public static double ALIGN_kP = 0.025;
    public static double ALIGN_kI = 0.0;
    public static double ALIGN_kD = 0.0;
    public static double ALIGN_tY = -2.0;
    public static double ALIGN_ERROR_TOLERANCE = 1;

    // Constants
    public static double SLIDE_ERROR_TOLERANCE = 15.0;
    public static double SLIDE_TIMEOUT = 1400;
    public static double ZEROING_TIMEOUT = 2000;
    public static double ZEROING_VELOCITY_ERROR = 5;

    // Positions
    public static double SLIDE_REST_POSITION = 10.0;
    public static double SLIDE_SAMPLE_COLLECT_POSITION = 1090.0;
    public static double SLIDE_CLOSE_SAMPLE_COLLECT_POSITION = 420;
    public static double SLIDE_SPECIMEN_COLLECT_POSITION = 350;
    public static double SLIDE_CHAMBER_POSITION = 430;
    public static double SLIDE_CHAMBER_SCORE_OFFSET = 350;
    public static double SLIDE_BASKET_POSITION = 1380;

    // Climb
    public static double SLIDE_CLIMB_POSITION = 550;
    public static double SLIDE_LATCH_POSITION = 220.0;
    public static double SLIDE_PULL_CLIMB_POSITION = 430.0;
    public static double SLIDE_ENGAGE_POSITION = 235.0;

    /**
     * PIVOT
     */

    // PID
    public static double PIVOT_kP = 0.0095;
    public static double PIVOT_kI = 0.0;
    public static double PIVOT_kD = 0.000045;
    public static double PIVOT_kF = 0.0;

    // Constants
    public static double PIVOT_ERROR_TOLERANCE = 5.0;
    public static double PIVOT_GEAR_RATIO = 40.0 / 26.0;
    public static double PIVOT_TIMEOUT = 1500;

    // Positions
    public static double PIVOT_REST_POSITION = 64;
    public static final double PIVOT_REST_TO_SCORE_OFFSET = 150.0;
    public static double PIVOT_SCORE_POSITION = PIVOT_REST_POSITION + PIVOT_REST_TO_SCORE_OFFSET;
    public static double PIVOT_CLIMBED_POSITION = PIVOT_SCORE_POSITION - 30;
}
