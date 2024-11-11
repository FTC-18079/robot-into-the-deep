package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Config
public class VisionConstants {
    // AprilTag camera position
    public static final Position cameraPose = new Position(DistanceUnit.MM, 8.0315, 5.3871 ,3.3625, 0);

    // Arducam lens intrinsics
    public static final double arducam_fx = 549.535;
    public static final double arducam_fy = 549.535;
    public static final double arducam_cx = 310.076;
    public static final double arducam_cy = 251.101;


    // Claw angles
    public static double[][] CLAW_ANGLES = {
            {0,   0.45},
            {45,  0.3},
            {90,  1.0},
            {135, 0.6},
            {180, 0.45},
            {225, 0.3},
            {270, 1.0},
            {315, 0.6},
            {360, 0.45}
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
