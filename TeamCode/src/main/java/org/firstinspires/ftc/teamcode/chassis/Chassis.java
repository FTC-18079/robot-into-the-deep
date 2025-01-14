package org.firstinspires.ftc.teamcode.chassis;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hydra;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.RobotStatus;
import org.firstinspires.ftc.teamcode.util.SubsystemIF;

public class Chassis extends SubsystemIF {
    Follower follower;
    Telemetry telemetry;
    boolean isFieldCentric;
    double m = 1.0;

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
        follower = new Follower(RobotMap.getInstance().getHardwareMap());
        follower.setPose(RobotStatus.robotPose);
    }

    @Override
    public void onTeleopInit() {
        telemetry = Hydra.getInstance().getTelemetry();
        follower = new Follower(RobotMap.getInstance().getHardwareMap());
        follower.setPose(RobotStatus.robotPose);
        follower.startTeleopDrive();
        setMaxPower(1.0);
    }

    // GETTERS

    public Pose getPoseEstimate() {
        return follower.getPose();
    }

    public boolean isBusy() {
        return follower.isBusy();
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
        follower.setTeleOpMovementVectors(fwd * m, str * m, rot * m, isFieldCentric);
    }

    public void resetHeading() {
        Pose oldPose = getPoseEstimate();
        follower.setPose(new Pose(oldPose.getX(), oldPose.getY()));
    }

    public void enableSlowMode() {
        m = 0.30;
    }

    public void disableSlowMode() {
        m = 1.0;
    }

    // PATHING

    public void followPath(Path path) {
        follower.followPath(path);
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
