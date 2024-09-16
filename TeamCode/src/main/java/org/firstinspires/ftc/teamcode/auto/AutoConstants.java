package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.util.RobotGlobal;

// Class for storing poses
public class AutoConstants {
    // Robot start poses
    public enum RobotStart {
        OBSERVATION_ZONE, CENTER, BASKET
    }

    // Starting poses
    public static final Pose OBVZONE_STARTING_POSE = new Pose(8, 60, 0);
    public static final Pose CENTER_STARTING_POSE = new Pose(8, 80, 0);
    public static final Pose BASKET_STARTING_POSE = new Pose(8, 108, 0);

    // Robot parking poses
    public enum ParkingPose {
        OBSERVATION_ZONE, SUBMERSIBLE
    }

    // Scoring poses
    public static final Pose BASKET_SCORE_POSE = new Pose(16, 128, Math.toRadians(135));
    public static final Pose OBV_PRELOAD_SCORE_POSE = new Pose(38, 60, Math.toRadians(0));
    public static final Pose CENTER_PRELOAD_SCORE_POSE = new Pose(38, 84, Math.toRadians(0));

    // Parking poses
    public static final Pose OBVZONE_PARKING_POSE = new Pose(16.2, 12, Math.toRadians(90));
    public static final Pose SUBMERSIBLE_PARKING_POSE = new Pose(60, 97, Math.toRadians(-90));

    // Blue Side Alliance-Specific Element Poses
    public static final Pose BLUE_LEFT_ALLIANCE_SAMPLE = new Pose(2.5+9.75+10.5, 4*24+1.5);
    public static final Pose BLUE_MID_ALLIANCE_SAMPLE = new Pose(2.5+9.75, 4*24+1.5);
    public static final Pose BLUE_RIGHT_ALLIANCE_SAMPLE = new Pose(2.5, 4*24+1.5);

    // Blue Side Neutral Element Poses
    public static final Pose BLUE_LEFT_NEUTRAL_SAMPLE = new Pose(144-2.5, 4*24+1.5);
    public static final Pose BLUE_MID_NEUTRAL_SAMPLE = new Pose(144-2.5-9.75, 4*24+1.5);
    public static final Pose BLUE_RIGHT_NEUTRAL_SAMPLE = new Pose(144-2.5-9.75-10.5, 4*24+1.5);

    // Red Side Alliance-Specific Element Poses
    public static final Pose RED_LEFT_ALLIANCE_SAMPLE = new Pose(144-2.5-9.75-10.5, 2*24-2.5);
    public static final Pose RED_MID_ALLIANCE_SAMPLE = new Pose(144-2.5-9.75, 2*24-2.5);
    public static final Pose RED_RIGHT_ALLIANCE_SAMPLE = new Pose(144-2.5, 2*24-2.5);

    // Red Side Neutral Element Poses
    public static final Pose RED_LEFT_NEUTRAL_SAMPLE = new Pose(2.5, 2*24-2.5);
    public static final Pose RED_MID_NEUTRAL_SAMPLE = new Pose(2.5+9.75, 2*24-2.5);
    public static final Pose RED_RIGHT_NEUTRAL_SAMPLE = new Pose(2.5+9.75+10.5, 2*24-2.5);

    public static Pose checkAlliance(Pose pose) {
        if (RobotGlobal.alliance == RobotGlobal.Alliance.RED) return toRed(pose);
        else return pose;
    }

    private static Pose toRed(Pose pose) {
        return new Pose(144 - pose.getX(), 144 - pose.getY(), pose.getHeading() + Math.toRadians(180));
    }
}
