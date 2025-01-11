package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.RobotStatus;

// Class for storing poses
@Config
public class AutoConstants {
    // Scoring poses
    public static double CHAMBER_X_POSITION = 31.25;
    public static double CHAMBER_RIGHT_Y_POSITION = 74;

    public static final Pose BASKET_SCORE_POSE = new Pose(17, 124, Math.toRadians(315));
    public static final Pose CHAMBER_LEFT_SCORE_POSE = new Pose(CHAMBER_X_POSITION, 80, Math.toRadians(180));
    public static final Pose CHAMBER_RIGHT_SCORE_POSE = new Pose(CHAMBER_X_POSITION, CHAMBER_RIGHT_Y_POSITION, Math.toRadians(180));

    // Parking poses
    public static final Pose OBVZONE_PARKING_POSE = new Pose(11, 36, Math.toRadians(270));
    public static final Pose ASCENT_PARKING_POSE = new Pose(60, 96, Math.toRadians(90));

    // Alliance alliance poses
    public static final Pose LEFT_ALLIANCE_SAMPLE = new Pose(46, 22.75);
    public static final Pose MIDDLE_ALLIANCE_SAMPLE = new Pose(46, 12.5);
    public static final Pose RIGHT_ALLIANCE_SAMPLE = new Pose(46, 2.25);

    // Neutral sample poses
    public static final Pose LEFT_NEUTRAL_SAMPLE = new Pose(46, 141.75);
    public static final Pose MIDDLE_NEUTRAL_SAMPLE = new Pose(46, 131.5);
    public static final Pose RIGHT_NEUTRAL_SAMPLE = new Pose(46, 121.25);

    // Specimen collect poses
    public static double WALL_COLLECT_X_POSITION = 25;
    public static double WALL_COLLECT_X_POSITION_2 = 22.5;
    public static double WALL_COLLECT_Y_POSITION = 35.5;

    public static Pose checkAlliance(Pose pose) {
        if (RobotStatus.alliance == RobotStatus.Alliance.RED) return toRed(pose);
        else return pose;
    }

    private static Pose toRed(Pose pose) {
        return new Pose(144 - pose.getX(), 144 - pose.getY(), pose.getHeading() + Math.toRadians(180));
    }
}
