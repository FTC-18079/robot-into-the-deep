package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

@Config
public class VisionConstants {
    // AprilTag camera position
    public static Vector2d cameraPose = new Vector2d(0, 0);

    // Arducam lens intrinsics
    public static final double arducam_fx = 0.0;
    public static final double arducam_fy = 0.0;
    public static final double arducam_cx = 0.0;
    public static final double arducam_cy = 0.0;

    // AprilTag position library
    public static AprilTagLibrary getAprilTagLibrary() {
        return new AprilTagLibrary.Builder()
                .addTag(11, "AudienceLeft",
                        4, new VectorF(0, 0, 0f), DistanceUnit.INCH,
                        new Quaternion(0f, 0f, 0f, 0f, 0))
                .addTag(12, "BlueAlliance",
                        4, new VectorF(0, 0, 0f), DistanceUnit.INCH,
                        new Quaternion(0f, 0f, 0f, 0f, 0))
                .addTag(13, "FarLeft",
                        4, new VectorF(0, 0, 0f), DistanceUnit.INCH,
                        new Quaternion(0f, 0f, 0f, 0f, 0))
                .addTag(14, "FarRight",
                        4, new VectorF(0, 0, 0f), DistanceUnit.INCH,
                        new Quaternion(0f, 0f, 0f, 0f, 0))
                .addTag(15, "RedAlliance",
                        4, new VectorF(0, 0, 0f), DistanceUnit.INCH,
                        new Quaternion(0f, 0f, 0f, 0f, 0))
                .addTag(16, "AudienceRight",
                        4, new VectorF(0, 0, 0f), DistanceUnit.INCH,
                        new Quaternion(0f, 0f, 0f, 0f, 0))
                .build();
    }
}
