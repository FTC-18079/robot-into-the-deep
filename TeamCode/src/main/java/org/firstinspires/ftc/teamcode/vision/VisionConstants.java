package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Config
public class VisionConstants {
    // AprilTag camera position
    public static final Position cameraPose = new Position(DistanceUnit.MM, 0, 0 ,0, 0);

    // Arducam lens intrinsics
    public static final double arducam_fx = 0.0;
    public static final double arducam_fy = 0.0;
    public static final double arducam_cx = 0.0;
    public static final double arducam_cy = 0.0;
}
