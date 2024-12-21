package org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers;

import static org.firstinspires.ftc.teamcode.vision.VisionConstants.arducam_cx;
import static org.firstinspires.ftc.teamcode.vision.VisionConstants.arducam_cy;
import static org.firstinspires.ftc.teamcode.vision.VisionConstants.arducam_fx;
import static org.firstinspires.ftc.teamcode.vision.VisionConstants.arducam_fy;

import android.util.Size;

import com.arcrobotics.ftclib.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Localizer;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.RobotStatus;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class AprilTagLocalizer extends Localizer {
    private Pose startPose;
    private double previousHeading;
    private double totalHeading;
    private long tagDetectTime;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private final boolean xAxisInverted = false; // TODO: Change if needed
    private final boolean yAxisInverted = false; // TODO: Change if needed
    private final Pose cameraOffset;
    private final Localizer secondaryLocalizer;

    public AprilTagLocalizer() {
        this(new Pose());
    }

    public AprilTagLocalizer(Pose startPose) {
        secondaryLocalizer = new OTOSLocalizer();

        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(arducam_fx, arducam_fy, arducam_cx, arducam_cy)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(RobotMap.getInstance().APRILTAG_CAMERA)
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(RobotStatus.liveView)
                .build();

        cameraOffset = new Pose(-8.0315, 0);

        setStartPose(startPose);
        totalHeading = 0;
        previousHeading = startPose.getHeading();
    }

    @Override
    public Pose getPose() {
        Pose pose = getVectorBasedOnTags();

        if (pose != null) {
            double x = pose.getX();
            double y = pose.getY();

            if (xAxisInverted) x *= -1;
            if (yAxisInverted) y *= -1;

            secondaryLocalizer.setPose(new Pose(x, y));

            return MathFunctions.addPoses(startPose, new Pose(x, y, secondaryLocalizer.getTotalHeading()));
        } else {
            return secondaryLocalizer.getPose();
        }
    }

    public Pose getVectorBasedOnTags() {
        ArrayList<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.isEmpty()) {
            return null;
        } else {
            tagDetectTime = detections.get(0).frameAcquisitionNanoTime;

            Vector2d posVector2d = detections.stream()
                    .map(detection -> getFCPosition(detection, secondaryLocalizer.getPose().getHeading(), cameraOffset))
                    .reduce(new Vector2d(), Vector2d::plus)
                    .div(detections.size());

            return new Pose(posVector2d.getX(), posVector2d.getY());
        }
    }

    public Vector2d getFCPosition(AprilTagDetection detection, double botHeading, Pose cameraOffset) {
        double x = detection.ftcPose.x - cameraOffset.getX();
        double y = detection.ftcPose.y - cameraOffset.getY();

        botHeading = -botHeading;

        double x2 = x * Math.cos(botHeading) + y * Math.sin(botHeading);
        double y2 = x * -Math.sin(botHeading) + y * Math.cos(botHeading);

        VectorF tagPose = AprilTagGameDatabase.getIntoTheDeepTagLibrary().lookupTag(detection.id).fieldPosition;

        return new Vector2d(tagPose.get(0) + y2 + 72, tagPose.get(1) + x2 + 72);
    }

    // Pedro stuff from here on out

    @Override
    public Pose getVelocity() {
        return secondaryLocalizer.getVelocity();
    }

    @Override
    public Vector getVelocityVector() {
        return getVelocity().getVector();
    }

    @Override
    public void setStartPose(Pose pose) {
        startPose = pose;
    }

    // TODO: implement this
    @Override
    public void setPose(Pose pose) {

    }

    @Override
    public void update() {
        secondaryLocalizer.update();
        totalHeading = secondaryLocalizer.getTotalHeading();
    }

    @Override
    public double getTotalHeading() {
        return totalHeading;
    }

    @Override
    public double getForwardMultiplier() {
        return secondaryLocalizer.getForwardMultiplier();
    }

    @Override
    public double getLateralMultiplier() {
        return secondaryLocalizer.getLateralMultiplier();
    }

    @Override
    public double getTurningMultiplier() {
        return secondaryLocalizer.getTurningMultiplier();
    }
}
