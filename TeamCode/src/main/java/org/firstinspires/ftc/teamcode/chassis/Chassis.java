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

public class Chassis extends SubsystemBase {
    Follower follower;
    Telemetry telemetry;
    boolean isRobotCentric;

    private static Chassis INSTANCE = null;

    public static Chassis getInstance() {
        if (INSTANCE == null) INSTANCE = new Chassis();
        return INSTANCE;
    }

    public void resetInstance() {
        INSTANCE = null;
    }

    private Chassis() {
        this.telemetry = RobotCore.getTelemetry();
        isRobotCentric = false;
        follower = new Follower(new Pose());
    }

    public void setPosition(Pose pose) {
        follower.setPose(pose);
    }

    public void setDrivePowers(double fwd, double str, double rot) {
        follower.setTeleOpMovementVectors(fwd, str, rot, isRobotCentric);
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

    public void startTeleopDrive() {
        follower.startTeleopDrive();
    }

    @Override
    public void periodic() {
        follower.update();
        telemetry.addData("Robot Centric", isRobotCentric);
        telemetry.addData("Path exists", follower.getCurrentPath() != null);

        Drawing.drawDebug(follower);

        // Draw robot
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.fieldOverlay().setStroke("#4CAF50");
//        Drawing.drawRobotOnCanvas(packet.fieldOverlay(), getPoseEstimate());
//        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

}
