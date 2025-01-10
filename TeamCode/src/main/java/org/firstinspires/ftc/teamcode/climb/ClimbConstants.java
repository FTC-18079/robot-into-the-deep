package org.firstinspires.ftc.teamcode.climb;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ClimbConstants {
    // PID

    public static double kP = 1.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.2;

    // Constants

    public static double CLIMB_ERROR_TOLERANCE = 0.0;

    // POSITIONS

    public static double CLIMB_LATCH_POSITION = 700.0;
    public static double CLIMB_UNSPOOLED_POSITION = -5950;
    public static double CLIMB_PIVOT_POSITION = 800.0;
    public static double CLIMB_IN_POSITION = 900;
}
