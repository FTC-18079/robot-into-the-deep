package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hydra;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.RobotStatus;
import org.firstinspires.ftc.teamcode.util.ATLivestream;
import org.firstinspires.ftc.teamcode.util.SubsystemIF;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import static org.firstinspires.ftc.teamcode.vision.VisionConstants.*;

import java.util.ArrayList;

public class ATVision extends SubsystemIF {
    private Telemetry telemetry;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private static final ATVision INSTANCE = new ATVision();

    public static ATVision getInstance() {
        return INSTANCE;
    }

    // INITIALIZE

    @Override
    public void onAutonomousInit() {
        telemetry = Hydra.getInstance().getTelemetry();

        makeProcessor();
        makePortal();
    }

    @Override
    public void onTeleopInit() {
        telemetry = Hydra.getInstance().getTelemetry();

        makeProcessor();
        makePortal();
    }

    @Override
    public void onDisabledInit() {
        closePortal();
    }

    private void makeProcessor() {
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(arducam_fx, arducam_fy, arducam_cx, arducam_cy)
                .setDrawAxes(true)
                .setCameraPose(CAMERA_POSE, CAMERA_ROTATION)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .build();
        aprilTag.setDecimation(3);
    }

    private void makePortal() {
        VisionPortal.Builder portalBuilder = new VisionPortal.Builder()
                .setCamera(RobotMap.getInstance().APRILTAG_CAMERA)
                .setCameraResolution(CAMERA_RESOLUTION)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(RobotStatus.liveView)
                .setAutoStopLiveView(true)
                .addProcessor(aprilTag);

        if (RobotStatus.liveView) {
            portalBuilder.enableLiveView(true);
            portalBuilder.addProcessor(new ATLivestream());
        }

        visionPortal = portalBuilder.build();
    }

    // GETTERS

    public ArrayList<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    // SETTERS

    public void stopStreaming() {
        visionPortal.stopStreaming();
    }

    public void resumeStreaming() {
        visionPortal.resumeStreaming();
    }

    public void closePortal() {
        visionPortal.close();
    }
}
