package org.firstinspires.ftc.teamcode.collector;

import com.acmerobotics.dashboard.config.Config;

@Config
public class CollectorConstants {
    public static double MAX_DELTAV = 400.0;

    // Slide poses
    public static int SLIDE_MAX_POS = 1400;
    public static int SLIDE_STOW_POS = 110;
    public static int SLIDE_COLLECTING_POS = 1300;
    public static int SLIDE_COLLECT_POS = 850;
    public static int SLIDE_PASSTHROUGH_POS = 300;
    public static int VELOCITY_THRESHOLD = 15;
    public static int ERROR_TOLERANCE = 10;

    public static double kP = 5.15;
    public static double kI = 0.0;
    public static double kD = 0.1;

    // Deploy poses
    public static double DEPLOY_STOW_POS = 0.99;
    public static double DEPLOY_SEEKING_POS = 0.72;
    public static double DEPLOY_COLLECT_POS = 0.03;

    // Pivot poses
    public static double PIVOT_ZERO_POS = 0.0;
    public static double PIVOT_MAX_POS = 1.0;
    public static double PIVOT_PASSTHROUGH_POS = 0.5;

    // Intake poses
    public static double INTAKE_GRAB_POS = 0.50;
    public static double INTAKE_RELEASE_POS = 1.0;

    public static double toTicksPerSec(double rpm) {
        return (rpm * 560.0) / 60.0;
    }

    public static double toRPM(double ticksPerSec) {
        return (ticksPerSec * 60.0) / 560.0;
    }
}
