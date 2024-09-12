package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.auto.AutoConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public class RobotGlobal {
    public enum Alliance {
        RED, BLUE, NONE
    }

    public static long delayMs = 0;
    public static Alliance alliance = Alliance.NONE;
    public static boolean liveView = false;
    public static Pose robotPose = new Pose();
    public static AutoConstants.ParkingPose parkingPose = AutoConstants.ParkingPose.SUBMERSIBLE;
}
