package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public class RobotGlobal {
    public enum Alliance {
        RED, BLUE, NONE
    }

    public static Alliance alliance = Alliance.NONE;
    public static Pose robotPose = new Pose();
}
