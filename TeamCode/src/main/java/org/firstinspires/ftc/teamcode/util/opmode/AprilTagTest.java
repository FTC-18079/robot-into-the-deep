package org.firstinspires.ftc.teamcode.util.opmode;

import static org.firstinspires.ftc.teamcode.vision.VisionConstants.arducam_fx;
import static org.firstinspires.ftc.teamcode.vision.VisionConstants.arducam_fy;
import static org.firstinspires.ftc.teamcode.vision.VisionConstants.arducam_cx;
import static org.firstinspires.ftc.teamcode.vision.VisionConstants.arducam_cy;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "AprilTag Test", group = "Tests")
public class AprilTagTest extends OpMode {
    AprilTagProcessor tagProcessor;
    VisionPortal portal;

    @Override
    public void init() {
        RobotMap.getInstance().init(hardwareMap);

        tagProcessor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(arducam_fx, arducam_fy, arducam_cx, arducam_cy)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(RobotMap.getInstance().APRILTAG_CAMERA)
                .addProcessor(tagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .build();
    }

    @Override
    public void loop() {

    }
}
