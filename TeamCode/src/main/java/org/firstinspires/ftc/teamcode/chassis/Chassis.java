package org.firstinspires.ftc.teamcode.chassis;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public class Chassis extends SubsystemBase {
    Follower follower;
    Telemetry telemetry;
    boolean isRobotCentric;

    public Chassis(RobotCore robot, Pose initialPose) {
        this.telemetry = robot.getTelemetry();
        isRobotCentric = false;
        follower = new Follower(initialPose);
    }

    public void setDrivePowers(double fwd, double str, double rot) {
        follower.setTeleOpMovementVectors(fwd, str, rot);
    }

    public Pose getPoseEstimate() {
        return follower.getPose();
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

    public void startTeleOpDrive() {
        follower.startTeleopDrive();
    }

    public void periodic() {
        follower.update();
        telemetry.addData("Robot Centric", isRobotCentric);
    }

}
