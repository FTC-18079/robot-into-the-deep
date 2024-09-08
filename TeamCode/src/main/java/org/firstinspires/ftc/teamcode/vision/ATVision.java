package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.util.ATLivestream;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import static org.firstinspires.ftc.teamcode.vision.VisionConstants.arducam_fx;
import static org.firstinspires.ftc.teamcode.vision.VisionConstants.arducam_fy;
import static org.firstinspires.ftc.teamcode.vision.VisionConstants.arducam_cx;
import static org.firstinspires.ftc.teamcode.vision.VisionConstants.arducam_cy;
import static org.firstinspires.ftc.teamcode.vision.VisionConstants.cameraPose;

import android.util.Size;

import java.util.List;

public class ATVision {
    private AprilTagProcessor processor;
    private VisionPortal portal;
    public AprilTagDetection lastDetection;

    public final ATLivestream livestream = new ATLivestream();

    public ATVision(boolean liveView) {
        // Create AT Processor
        processor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(arducam_fx, arducam_fy, arducam_cx, arducam_cy)
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setCameraPose(cameraPose, new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, 0))
                .build();

        // Create Vision Portal
        portal = new VisionPortal.Builder()
                .setCamera(RobotMap.getInstance().APRILTAG_CAMERA)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(processor)
                .addProcessor(livestream)
                .enableLiveView(liveView)
                .build();
    }

    public VisionPortal.CameraState getCameraState() {
        return portal.getCameraState();
    }

    public float getFPS() {
        return portal.getFps();
    }

    public List<AprilTagDetection> getDetections() {
        return processor.getDetections();
    }

    public Pose getVectorFromTags(double botHeading) {
        List<AprilTagDetection> currentDetections = getDetections();
        int realDetections = 0;
        Pose averagePose = new Pose();
        if (currentDetections.isEmpty()) return null;
        Pose robotPos;

        // Loop through detection list and calculate robot pose from each tag
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                robotPos = calculateRobotPose(detection, botHeading);

                // we're going to get the average here by adding them all up and dividingA the number of detections
                // we do this because the backdrop has 3 tags, so we get 3 positions
                // hopefully by averaging them we can get a more accurate position
                lastDetection = detection;
                averagePose.add(robotPos);
                realDetections++;
            }
        }

        averagePose.scalarDivide(realDetections);
        return averagePose;
    }

    public Pose toPose(VectorF vectorF) {
        return new Pose(vectorF.get(0), vectorF.get(1));
    }

    public double quaternionToHeading(Quaternion q) {
        return Math.atan2(2.0 * (q.z * q.w + q.x * q.y) , - 1.0 + 2.0 * (q.w * q.w + q.x * q.x)) - Math.toRadians(270);
    }

    public Pose calculateRobotPose(AprilTagDetection detection, double botHeading) {
        // Calculate robot coordinates
        double x = detection.ftcPose.x;
        double y = detection.ftcPose.y;
        botHeading = -botHeading;

        // Rotate coordinates to be field-centric
        double x2 = x * Math.cos(botHeading) + y * Math.sin(botHeading);
        double y2 = x * -Math.sin(botHeading) + y * Math.cos(botHeading);

        // Get tag pose from tag library
        VectorF tagPose = AprilTagGameDatabase.getIntoTheDeepTagLibrary().lookupTag(detection.id).fieldPosition;
        Pose newTagPose = toPose(tagPose);
        newTagPose.add(new Pose(72, 72)); // Convert to pedro coordinates

        return new Pose(newTagPose.getX() + x2, newTagPose.getY() + y2);
    }
}
