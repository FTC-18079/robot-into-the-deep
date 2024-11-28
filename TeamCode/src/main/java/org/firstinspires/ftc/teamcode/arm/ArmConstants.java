package org.firstinspires.ftc.teamcode.arm;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmConstants {
    /**
     * SLIDES
     */

    // PID
    public static double SLIDE_kP = 0.025;
    public static double SLIDE_kI = 0.0;
    public static double SLIDE_kD = 0.00025;
    public static double SLIDE_kF = 0.15;

    // Alignment PID
    public static double ALIGN_kP = 0.025;
    public static double ALIGN_kI = 0.0;
    public static double ALIGN_kD = 0.0;

    // Constants
    public static double SLIDE_ERROR_TOLERANCE = 35.0;
    public static double ALIGN_ERROR_TOLERANCE = 2;
    public static double SLIDE_TIMEOUT = 1500;

    // Positions
    public static double SLIDE_REST_POSITION = 20.0;
    public static double SLIDE_SAMPLE_COLLECT_POSITION = 1150.0;
    public static double SLIDE_SPECIMEN_COLLECT_POSITION = 350;
    public static double SLIDE_CHAMBER_POSITION = SLIDE_REST_POSITION;
    public static double SLIDE_BASKET_POSITION = 1500.0;

    /**
     * PIVOT
     */

    // PID
    public static double PIVOT_kP = 0.003;
    public static double PIVOT_kI = 0.0;
    public static double PIVOT_kD = 0.00008;
    public static double PIVOT_kF = 0.0;

    // Constants
    public static double PIVOT_ERROR_TOLERANCE = 100.0;
    public static double PIVOT_GEAR_RATIO = 40.0 / 15.0;
    public static double PIVOT_COUNTS_PER_REVOLUTION = 8192.0 * PIVOT_GEAR_RATIO;
    public static double PIVOT_TIMEOUT = 1900;

    // Positions
    public static double PIVOT_STARTING_ANGLE = 10.0;
    public static double PIVOT_STARTING_POS = (PIVOT_STARTING_ANGLE / 360.0) * PIVOT_COUNTS_PER_REVOLUTION;

    public static double PIVOT_REST_POSITION = 0.0;
    public static double PIVOT_FUDGE = 6.5;
    public static double PIVOT_SCORE_POSITION = ((90.0 + PIVOT_FUDGE) / 360.0) * PIVOT_COUNTS_PER_REVOLUTION; // TODO: remove the extra 5 once pivot is fixed
    public static double PIVOT_AUTO_ASCENT_POSITION = (65.0 / 360.0) * PIVOT_COUNTS_PER_REVOLUTION;
}
