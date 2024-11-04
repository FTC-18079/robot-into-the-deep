package org.firstinspires.ftc.teamcode.arm;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmConstants {
    public static class Slide {
        // PID coefficients
        public double kP = 0.001;
        public double kI = 0.0;
        public double kD = 0.0;
        public double kF = 0.0;

        public double alignP = 0.02;
        public double alignI = 0.0;
        public double alignD = 0.0;

        // Constants
        public double ERROR_TOLERANCE = 5.0;
    }

    public static class Pivot {
        // PID coefficients
        public double kP = 0.001;
        public double kI = 0.0;
        public double kD = 0.00005;
        public double kF = 0.0;

        // Constants
        public double ERROR_TOLERANCE = 5.0;
        public double GEAR_RATIO = 40.0 / 15.0;
        public double COUNTS_PER_REVOLUTION = 8192.0 * GEAR_RATIO;
    }

    public static Slide SLIDE = new Slide();
    public static Pivot PIVOT = new Pivot();
}
