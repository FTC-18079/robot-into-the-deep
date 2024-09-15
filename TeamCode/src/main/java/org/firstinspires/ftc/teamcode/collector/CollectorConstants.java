package org.firstinspires.ftc.teamcode.collector;

import com.acmerobotics.dashboard.config.Config;

@Config
public class CollectorConstants {
    public static double MAX_DELTAV = 415.0;

    public static int MAX_SLIDE_POS = 1400;
    public static int MIN_SLIDE_POS = 10;
    public static int MIN_VELOCITY = 15;
    public static int ERROR_TOLERANCE = 20;

    public static double kP = 5.15;
    public static double kI = 0.0;
    public static double kD = 0.1;

    public static double DEPLOY_DOWN_POS = 0.0;
    public static double DEPLOY_STOW_POS = 0.9;

    public static double toTicksPerSec(double rpm) {
        return (rpm * 560.0) / 60.0;
    }

    public static double toRPM(double ticksPerSec) {
        return (ticksPerSec * 60.0) / 560.0;
    }
}
