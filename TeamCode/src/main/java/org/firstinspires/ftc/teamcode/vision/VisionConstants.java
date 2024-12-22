package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
public class VisionConstants {
    public static final Position CAMERA_POSE = new Position(DistanceUnit.INCH, 0, -7.7815 ,3.625, 0);
    public static final YawPitchRollAngles CAMERA_ROTATION = new YawPitchRollAngles(AngleUnit.DEGREES, 180, -90, 0, 0);

    public static final Size CAMERA_RESOLUTION = new Size(640, 480);

    public static final double arducam_fx = 549.535;
    public static final double arducam_fy = 549.535;
    public static final double arducam_cx = 310.076;
    public static final double arducam_cy = 251.101;

    public static float DECIMATION = 2;
    public static double MAXIMUM_APRILTAG_RANGE = 48.0;
    public static double APRILTAG_REFRESH_RATE = 30.0;

    // Claw angles
    public static double[][] CLAW_ANGLES = {
            {0.0,   0.50},
            {22.5,  0.63},
            {45.0,  0.75},
            {67.5,  0.85},
            {90.0,  1.00},
            {112.5, 0.20},
            {135.0, 0.23},
            {157.5, 0.37},
            {180.0, 0.50},
            {202.5, 0.63},
            {225.0, 0.75},
            {247.5, 0.85},
            {270.0, 1.00},
            {292.5, 0.20},
            {315.0, 0.23},
            {337.5, 0.37},
            {360.0, 0.50}
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
