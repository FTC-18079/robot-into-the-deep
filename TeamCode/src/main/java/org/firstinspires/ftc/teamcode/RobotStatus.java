package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.autonomous.AutoConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public class RobotStatus {
    public enum Alliance {
        NONE, RED, BLUE,
    }

    public enum RobotState {
        DISABLED, AUTONOMOUS_INIT, AUTONOMOUS_ENABLED, TELEOP_INIT, TELEOP_ENABLED
    }

    public static long delayMs = 0;
    public static Alliance alliance = Alliance.NONE;
    public static RobotState robotState = RobotState.DISABLED;
    public static boolean liveView = false;
    public static Pose robotPose = new Pose();
    public static AutoConstants.ParkingLocation parkingLocation = AutoConstants.ParkingLocation.ASCENT_ZONE;

    public static void resetValues() {
        delayMs = 0;
        alliance = Alliance.NONE;
        robotState = RobotState.DISABLED;
        liveView = false;
        robotPose = new Pose();
        parkingLocation = AutoConstants.ParkingLocation.ASCENT_ZONE;
    }

    public static boolean isEnabled() {
        return robotState == RobotState.TELEOP_ENABLED || robotState == RobotState.AUTONOMOUS_ENABLED;
    }

    public static boolean isTeleop() {
        return robotState == RobotState.TELEOP_INIT || robotState == RobotState.TELEOP_ENABLED;
    }
}
