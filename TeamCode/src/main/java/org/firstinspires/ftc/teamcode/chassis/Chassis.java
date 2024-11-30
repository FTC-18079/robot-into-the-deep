package org.firstinspires.ftc.teamcode.chassis;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Drawing;
import org.firstinspires.ftc.teamcode.util.RobotGlobal;
import org.firstinspires.ftc.teamcode.util.SubsystemIF;

public class Chassis extends SubsystemBase implements SubsystemIF {
    Follower follower;
    Telemetry telemetry;
    boolean isRobotCentric;
    double m = 1.0;

    private static final Chassis INSTANCE = new Chassis();

    public static Chassis getInstance() {
        return INSTANCE;
    }

    private Chassis() {
        isRobotCentric = false;
        follower = new Follower(new Pose());
        this.telemetry = RobotCore.getTelemetry();
    }

    // INITIALIZE

    @Override
    public void onTeleopInit() {
        follower.initialize(RobotGlobal.robotPose);
        follower.startTeleopDrive();
        setMaxPower(1.0);
    }

    public void setMaxPower(double power) {
        follower.setMaxPower(power);
    }

    public void setPosition(Pose pose) {
        follower.setPose(pose);
    }

    public void setDrivePowers(double fwd, double str, double rot) {
        follower.setTeleOpMovementVectors(fwd * m, str * m, rot * m, isRobotCentric);
    }

    public Pose getPoseEstimate() {
        return follower.getPose();
    }

    public void followPath(Path path) {
        follower.followPath(path);
    }

    public void breakFollowing() {
        follower.breakFollowing();
    }

    public boolean isBusy() {
        return follower.isBusy();
    }

    public boolean isRobotCentric() {
        return isRobotCentric;
    }

    public void toggleRobotCentric() {
        isRobotCentric = !isRobotCentric;
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

    @Override
    public void periodic() {
        follower.update();
        RobotGlobal.robotPose = follower.getPose();

        telemetry.addLine();
        telemetry.addData("Robot Centric", isRobotCentric);
        telemetry.addData("Path exists", follower.getCurrentPath() != null);
    }

}
