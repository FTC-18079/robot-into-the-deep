package org.firstinspires.ftc.teamcode.chassis;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public class Chassis extends SubsystemBase {
    Follower follower;
    Telemetry telemetry;
    boolean isRobotCentric;

    public Chassis(RobotCore robot, Pose initialPose) {
//        this.telemetry = robot.getTelemetry();
        isRobotCentric = false;
        follower = new Follower(initialPose);
    }
}
