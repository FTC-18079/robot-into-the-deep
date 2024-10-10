package org.firstinspires.ftc.teamcode.collector;

import com.acmerobotics.dashboard.config.Config;

@Config
public class CollectorConstants {
    public static double MAX_DELTAV = 415.0;

    public static int MAX_SLIDE_POS = 1400;
    public static int MIN_SLIDE_POS = 10;
    public static int MIN_VELOCITY = 15;
    public static int SLIDE_PASSTHROUGH_POS = 0;
    public static int ERROR_TOLERANCE = 20;

    public static double kP = 5.15;
    public static double kI = 0.0;
    public static double kD = 0.1;

    public static double DEPLOY_STOW_POS = 0.9;
    public static double DEPLOY_TARGETING_POS = 0.4;
    public static double DEPLOY_COLLECTING_POS = 0.1;
    public static double DEPLOY_COLLECT_POS = 0.0;

    public static double PIVOT_ZERO_POS = 0.0;
    public static double PIVOT_MAX_POS = 1.0;
    public static double PIVOT_PASSTHROUGH_POS = 0.5;

    public static double INTAKE_GRAB_POS = 0.55;
    public static double INTAKE_RELEASE_POS = 1.0;

    public static double[] RED_RANGE = {10.0, 35.0};
    public static double[] YELLOW_RANGE = {60.0, 95.0};
    public static double[] BLUE_RANGE = {215.0, 250.0};

    public static double toTicksPerSec(double rpm) {
        return (rpm * 560.0) / 60.0;
    }

    public static double toRPM(double ticksPerSec) {
        return (ticksPerSec * 60.0) / 560.0;
    }
}
