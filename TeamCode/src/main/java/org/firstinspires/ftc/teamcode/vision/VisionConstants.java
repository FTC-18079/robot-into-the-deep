package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Config
public class VisionConstants {
    // AprilTag camera position
    public static final Position cameraPose = new Position(DistanceUnit.INCH, 8.0315, 5.3871 ,3.3625, 0);

    // Arducam lens intrinsics
    public static final double arducam_fx = 549.535;
    public static final double arducam_fy = 549.535;
    public static final double arducam_cx = 310.076;
    public static final double arducam_cy = 251.101;


    // Claw angles
    public static double[][] CLAW_ANGLES = {
            {0.0,   0.43},
            {22.5,  0.33},
            {45.0,  0.25},
            {67.5,  0.15},
            {90.0,  1.00},
            {112.5, 0.70},
            {135.0, 0.60},
            {157.5, 0.55},
            {180.0, 0.43},
            {202.5, 0.33},
            {225.0, 0.25},
            {247.5, 0.15},
            {270.0, 1.00},
            {292.5, 0.70},
            {315.0, 0.60},
            {337.5, 0.55},
            {360.0, 0.43}
    };

    public static double getNearestClawAngle(double angle) {
        double smallestError = 1000;
        int index = 0;
        for (int i = 0; i < CLAW_ANGLES.length; i++) {
            double error = Math.abs(angle - CLAW_ANGLES[i][0]);
            if (error < smallestError) {
                smallestError = error;
                index = i;
            }
        }
        return CLAW_ANGLES[index][1];
    }
}
