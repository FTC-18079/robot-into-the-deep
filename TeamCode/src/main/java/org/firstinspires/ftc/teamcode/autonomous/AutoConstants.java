package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.util.RobotGlobal;

// Class for storing poses
public class AutoConstants {
    // Robot start poses
    public enum RobotStart {
        OBSERVATION_ZONE, CENTER, BASKET
    }

    // Starting poses
    public static final Pose OBVZONE_STARTING_POSE = new Pose(8, 80, 0);

    // Robot parking poses
    public enum ParkingLocation {
        OBSERVATION_ZONE, ASCENT_ZONE
    }

    // Scoring poses
    public static final Pose BASKET_SCORE_POSE = new Pose(14, 126, Math.toRadians(315));
    public static final Pose CHAMBER_LEFT_SCORE_POSE = new Pose(32, 80, Math.toRadians(180));
    public static final Pose CHAMBER_RIGHT_SCORE_POSE = new Pose(32, 80, Math.toRadians(180));

    // Parking poses
    public static final Pose OBVZONE_PARKING_POSE = new Pose(14, 10, Math.toRadians(0));
    public static final Pose ASCENT_PARKING_POSE = new Pose(60, 98, Math.toRadians(90));

    // Alliance alliance poses
    public static final Pose LEFT_ALLIANCE_SAMPLE = new Pose(46, 22.75);
    public static final Pose MIDDLE_ALLIANCE_SAMPLE = new Pose(46, 12.5);
    public static final Pose RIGHT_ALLIANCE_SAMPLE = new Pose(46, 2.25);

    // Alliance sample poses
    public static final Pose LEFT_NEUTRAL_SAMPLE = new Pose(46, 141.75);
    public static final Pose MIDDLE_NEUTRAL_SAMPLE = new Pose(46, 131.5);
    public static final Pose RIGHT_NEUTRAL_SAMPLE = new Pose(46, 121.25);

    public static Pose checkAlliance(Pose pose) {
        if (RobotGlobal.alliance == RobotGlobal.Alliance.RED) return toRed(pose);
        else return pose;
    }

    private static Pose toRed(Pose pose) {
        return new Pose(144 - pose.getX(), 144 - pose.getY(), pose.getHeading() + Math.toRadians(180));
    }
}
