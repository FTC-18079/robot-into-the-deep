package org.firstinspires.ftc.teamcode.chassis;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hydra;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.RobotStatus;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.util.SubsystemIF;

public class Chassis extends SubsystemIF {
    private Follower follower;
    private Telemetry telemetry;
    private boolean isFieldCentric;
    private double velocityMultiplier = 1.0;

    private static final Chassis INSTANCE = new Chassis();

    public static Chassis getInstance() {
        return INSTANCE;
    }

    private Chassis() {
        isFieldCentric = true;
    }

    // INITIALIZE

    @Override
    public void onAutonomousInit() {
        telemetry = Hydra.getInstance().getTelemetry();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(RobotMap.getInstance().getHardwareMap());
        follower.setPose(RobotStatus.robotPose);
    }

    @Override
    public void onTeleopInit() {
        telemetry = Hydra.getInstance().getTelemetry();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(RobotMap.getInstance().getHardwareMap());
        follower.setPose(RobotStatus.robotPose);
        follower.startTeleopDrive();
        setMaxPower(1.0);
    }

    // GETTERS

    public Follower getFollower() {
        return follower;
    }

    public Pose getPoseEstimate() {
        return follower.getPose();
    }

    public boolean isBusy() {
        return follower.isBusy();
    }

    public boolean isRobotStuck() {
        return follower.isRobotStuck();
    }

    public boolean isFieldCentric() {
        return isFieldCentric;
    }

    // SETTERS

    public void setMaxPower(double power) {
        follower.setMaxPower(power);
    }

    public void setPosition(Pose pose) {
        follower.setPose(pose);
    }

    public void setDriveVectors(double fwd, double str, double rot) {
        follower.setTeleOpMovementVectors(fwd * velocityMultiplier, str * velocityMultiplier, rot * velocityMultiplier, !isFieldCentric);
    }

    public void resetHeading() {
        Pose oldPose = getPoseEstimate();
        follower.setPose(new Pose(oldPose.getX(), oldPose.getY()));
    }

    public void setVelocityMultiplier(double multiplier) {
        velocityMultiplier = MathFunctions.clamp(multiplier, 0d, 1d);
    }

    public void enableSlowMode() {
        setVelocityMultiplier(0.25);
    }

    public void disableSlowMode() {
        setVelocityMultiplier(1.0);
    }

    // PATHING

    public void followPath(Path path) {
        follower.followPath(path);
    }

    public void followPath(PathChain path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
    }

    public void breakFollowing() {
        follower.breakFollowing();
    }

    public void toggleFieldCentric() {
        isFieldCentric = !isFieldCentric;
    }

    // PERIODIC

    @Override
    public void periodic() {
        follower.update();
        RobotStatus.robotPose = follower.getPose();

        telemetry.addData("Field Centric", isFieldCentric);
        telemetry.addData("Path exists", follower.getCurrentPath() != null);
    }

}
