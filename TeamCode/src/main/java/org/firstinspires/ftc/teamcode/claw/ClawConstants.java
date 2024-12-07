package org.firstinspires.ftc.teamcode.claw;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ClawConstants {
    public static long JOINT_DELAY = 120;
    public static long COLLECT_DELAY = 30;
    public static long GRAB_DELAY = 400;

    public static ClawState REST_STATE = new ClawState(1.0, 1.0, 0.95, 0.0);

    // SAMPLE POSITIONS
    public static double SAMPLE_COLLECT_JOINT_ONE_POS = 0.2;
    public static double SAMPLE_COLLECT_JOINT_TWO_POS = 0.25;

    public static ClawState SAMPLE_COLLECTING_STATE = new ClawState(0.0, 1.0, 0.55, 1.0);
    public static ClawState SAMPLE_SCORING_STATE = new ClawState(1.0, 1.0, 0.5, 0.2);

    // SPECIMEN POSITIONS
    public static ClawState SPECIMEN_COLLECT_STATE = new ClawState(0.0, 1.0, 0.4, 0.0);
    public static ClawState SPECIMEN_SCORING_STATE = new ClawState(1.0, 0.0, 0.62, 0.25);
    public static ClawState SPECIMEN_SCORE_STATE = new ClawState(1.0, 0.0, 0.95, 0.25);

}
