package org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers;

import android.util.Log;

import com.arcrobotics.ftclib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Localizer;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.vision.ATVision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class AprilTagLocalizer extends Localizer {
    private final ATVision atVision = ATVision.getInstance();

    private Pose startPose;
    private Pose tagPose;
    private double previousHeading;
    private double totalHeading;
    private long tagDetectTime;

    private final Localizer secondaryLocalizer;

    public AprilTagLocalizer() {
        this(new Pose());
    }

    public AprilTagLocalizer(Pose startPose) {
        secondaryLocalizer = new OTOSLocalizer();
        setStartPose(startPose);
        totalHeading = 0;
        previousHeading = startPose.getHeading();
    }

    @Override
    public Pose getPose() {
        if (tagPose != null) {
            double x = tagPose.getX();
            double y = tagPose.getY();

            Pose newPose = MathFunctions.addPoses(startPose, new Pose(x, y, secondaryLocalizer.getPose().getHeading()));
            secondaryLocalizer.setPose(newPose);

            return newPose;
        } else {
            return secondaryLocalizer.getPose();
        }
    }

    private boolean isPoseValid(Pose pose) {
        // Don't accept if it's too stale
        if ((double) tagDetectTime > 100) {
            Log.i("AprilTagLocalizer", "===============Stale tag! (" + tagDetectTime + "ms) Reverting to secondary localizer.===============");
            return false;
        }
        // Don't accept tag if pose is out of field
        if (pose.getX() > 144 || pose.getY() > 144 || pose.getX() < 0 || pose.getY() < 0) {
            Log.i("AprilTagLocalizer", "===============Out of field! (" + pose.getX() + ", " + pose.getY() + ") Reverting to secondary localizer.===============");
            return false;
        }
        return true;
    }

    private Pose getPoseBasedOnTags(ArrayList<AprilTagDetection> detections) {
        if (detections.isEmpty()) {
            return null;
        } else {
            tagDetectTime = detections.get(0).frameAcquisitionNanoTime;

            Vector2d posVector2d = detections.stream()
                    .map(detection -> getBotVector(detection))
                    .reduce(new Vector2d(), Vector2d::plus)
                    .div(detections.size());

            return new Pose(posVector2d.getX(), posVector2d.getY());
        }
    }

    private Vector2d getBotVector(AprilTagDetection detection) {
        return new Vector2d(-detection.robotPose.getPosition().y + 72, detection.robotPose.getPosition().x + 72);
    }

    // Pedro stuff from here on out

    @Override
    public Pose getVelocity() {
        return secondaryLocalizer.getVelocity();
    }

    @Override
    public Vector getVelocityVector() {
        return secondaryLocalizer.getVelocityVector();
    }

    @Override
    public void setStartPose(Pose pose) {
        startPose = pose;
    }

    @Override
    public void setPose(Pose pose) {
        secondaryLocalizer.setPose(pose);
    }

    @Override
    public void update() {
        secondaryLocalizer.update();
        totalHeading = secondaryLocalizer.getTotalHeading();

        tagPose = getPoseBasedOnTags(atVision.getDetections());
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

    @Override
    public void resetIMU() throws InterruptedException {
        secondaryLocalizer.resetIMU();
    }
}
