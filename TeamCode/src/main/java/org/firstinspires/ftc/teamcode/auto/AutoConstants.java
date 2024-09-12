package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

// Class for storing poses
public class AutoConstants {
    // Robot start poses
    public enum RobotStart {
        OBSERVATION_ZONE, CENTER, BASKET
    }

    // Starting poses
    public static final Pose OBVZONE_STARTING_POSE = new Pose(9, 108, 0);
    public static final Pose CENTER_STARTING_POSE = new Pose(9, 80, 0);
    public static final Pose BASKET_STARTING_POSE = new Pose(9, 60, 0);

    public enum ParkingPose {
        OBSERVATION_ZONE, SUBMERSIBLE
    }

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
}
