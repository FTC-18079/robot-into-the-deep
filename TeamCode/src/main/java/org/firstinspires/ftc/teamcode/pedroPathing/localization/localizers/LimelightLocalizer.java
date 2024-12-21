package org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers;

import android.util.Log;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Localizer;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

/**
 *
 */
public class LimelightLocalizer extends Localizer {
    private Pose startPose;
    private Limelight3A limelight;
    private LLResult result;
    private double previousHeading;
    private double totalHeading;

    private final Localizer secondaryLocalizer;

    public LimelightLocalizer() {
        this(new Pose());
    }

    public LimelightLocalizer(Pose startPose) {
        secondaryLocalizer = new OTOSLocalizer();

        limelight = RobotMap.getInstance().LIMELIGHT;
        limelight.pipelineSwitch(4);
        limelight.setPollRateHz(125);
        limelight.start();

        setStartPose(startPose);
        totalHeading = 0;
        previousHeading = startPose.getHeading();
    }

    /**
     * This returns the current pose estimate.
     *
     * @return returns the current pose estimate as a Pose
     */
    // TODO: translate coordinate systems???
    // TODO: kalman filter? some way of fusing readings instead of just accepting it? black magic?
    @Override
    public Pose getPose() {
        // Check if result actually exists
        if (result != null && result.isValid() && result.getPipelineIndex() == 4) {
            Pose3D llPose = result.getBotpose_MT2();
            // Check that we accept the limelight pose
            if (llPose != null && isPoseValid(llPose)) {
                // Add pose to the starting position and return that
                Pose pose = MathFunctions.addPoses(startPose, new Pose(llPose.getPosition().x, llPose.getPosition().y, llPose.getOrientation().getYaw(AngleUnit.RADIANS)));
                secondaryLocalizer.setPose(pose);
                return pose;
            } else {
                // Take a snapshot of bad poses for debugging
                Log.i("LimelightLocalizer", "===============Error: Bad pose found! Saving snapshot.===============");
                limelight.captureSnapshot("bad_localizer_pose");
            }
        }
        return secondaryLocalizer.getPose();
    }

    public boolean isPoseValid(Pose3D pose) {
        // Don't accept pose if it's from >100 ms ago
        if (result.getStaleness() > 100) {
            Log.i("LimelightLocalizer", "===============Stale tag! Reverting to secondary localizer.===============");
            return false;
        }
        // Don't accept pose if heading is cooked
        if (secondaryLocalizer.getVelocityVector().getMagnitude() != 0 && secondaryLocalizer.getPose().getHeading() == 0 && previousHeading == 0) {
            Log.i("LimelightLocalizer", "===============No heading difference. Reverting to secondary localizer.===============");
            return false;
        }
        // Don't accept pose if it's outside the field
        if (pose.getPosition().x > 144 || pose.getPosition().y > 144 || pose.getPosition().x < 0 || pose.getPosition().y < 0) {
            Log.i("LimelightLocalizer", "===============Out of field. Reverting to secondary localizer.===============");
            return false;
        }
        // Don't accept pose if no heading is present
        if (pose.getOrientation().getYaw() == 0 && pose.getOrientation().getPitch() == 0 && pose.getOrientation().getRoll() == 0) {
            Log.i("LimelightLocalizer", "===============No heading found. Reverting to secondary localizer.===============");
        }
        return true;
    }

    /**
     * This returns the current velocity estimate from the Localizer.
     *
     * @return returns the velocity as a Pose object.
     */
    @Override
    public Pose getVelocity() {
        return secondaryLocalizer.getVelocity();
    }

    /**
     * This returns the current velocity estimate from the Localizer as a Vector.
     *
     * @return returns the velocity as a Vector.
     */
    @Override
    public Vector getVelocityVector() {
        return secondaryLocalizer.getVelocityVector();
    }

    /**
     * This sets the start pose of the Localizer. Changing the start pose should move the robot as if
     * all its previous movements were displacing it from its new start pose.
     *
     * @param setStart the new start pose
     */
    @Override
    public void setStartPose(Pose setStart) {
        startPose = setStart;
    }

    /**
     * This sets the current pose estimate of the Localizer. Changing this should just change the
     * robot's current pose estimate, not anything to do with the start pose.
     *
     * @param setPose the new current pose estimate
     */
    @Override
    public void setPose(Pose setPose) {
        secondaryLocalizer.setPose(setPose);
    }

    /**
     * This calls an update to the Localizer, updating the current pose estimate and current velocity
     * estimate.
     */
    @Override
    public void update() {
        secondaryLocalizer.update();
        totalHeading = secondaryLocalizer.getTotalHeading();
        result = limelight.getLatestResult();
        limelight.updateRobotOrientation(secondaryLocalizer.getPose().getHeading());
    }

    /**
     * This returns how far the robot has turned in radians, in a number not clamped between 0 and
     * 2 * pi radians. This is used for some tuning things and nothing actually within the following.
     *
     * @return returns how far the robot has turned in total, in radians.
     */
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
