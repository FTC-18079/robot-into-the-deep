package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

// Class for storing poses
public class AutoConstants {
    // Robot start poses
    public enum RobotStart {
        OBSERVATION_ZONE, CENTER, BASKET
    }

    public enum ParkingPose {
        OBSERVATION_ZONE, SUBMERSIBLE
    }

    public static final Pose EXAMPLE_START_POSE = new Pose();

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
