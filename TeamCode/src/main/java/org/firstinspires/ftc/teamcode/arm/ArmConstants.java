package org.firstinspires.ftc.teamcode.arm;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmConstants {
    public static class Slide {
        // PID coefficients
        public double kP = 1.0;
        public double kI = 0.0;
        public double kD = 0.0;
        public double kF = 0.0;

        // Constants
        public double ERROR_TOLERANCE = 5.0;
        public double TICKS_IN_DEGREES = 0.0;
    }

    public static class Pivot {
        // PID coefficients
        public double kP = 1.0;
        public double kI = 0.0;
        public double kD = 0.0;
        public double kF = 0.0;

        // Constants
        public double ERROR_TOLERANCE = 5.0;
        public double TICKS_IN_DEGREES = 0.0;
    }

    public static Slide SLIDE = new Slide();
    public static Pivot PIVOT = new Pivot();
}
