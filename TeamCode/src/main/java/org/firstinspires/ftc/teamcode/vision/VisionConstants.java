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
}
