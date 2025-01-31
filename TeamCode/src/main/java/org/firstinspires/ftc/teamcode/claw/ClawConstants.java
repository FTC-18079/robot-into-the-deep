package org.firstinspires.ftc.teamcode.claw;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ClawConstants {
    public static long PIVOT_DELAY = 10;
    public static long GRAB_DELAY = 225;

    public static double CLAW_CLOSE_POSITION = 0.4;
    public static double CLAW_OPEN_POSITION = 1.0;

    public static double JOINT_ONE_EJECT_POSITION = 0.6;

    public static ClawState INIT_STATE = new ClawState(CLAW_CLOSE_POSITION, 0.0, 0.0, 1.0);
    public static ClawState REST_STATE = new ClawState(CLAW_CLOSE_POSITION, 1.0, 1.0, 0.9);

    // SAMPLE POSITIONS
    public static double SAMPLE_COLLECT_JOINT_ONE_POS = 0.35;
    public static double SAMPLE_COLLECT_JOINT_TWO_POS = 0.7;

    public static ClawState SAMPLE_COLLECTING_STATE = new ClawState(CLAW_OPEN_POSITION, 0.95, 0.78, 0.0);
    public static ClawState SAMPLE_SCORING_STATE = new ClawState(CLAW_CLOSE_POSITION, 0.95, 0.7, 0.82);

    // SPECIMEN POSITIONS
    public static ClawState SPECIMEN_COLLECT_STATE = new ClawState(CLAW_OPEN_POSITION, 0.95, 0.55, 1.0);
    public static ClawState SPECIMEN_SCORING_STATE = new ClawState(CLAW_CLOSE_POSITION, 0.0, 0.8, 1.0);
    public static ClawState SPECIMEN_SCORE_STATE = new ClawState(CLAW_CLOSE_POSITION, 0.0, 0.9, 1.0);
    public static ClawState SPECIMEN_AUTO_SCORING_STATE = new ClawState(CLAW_CLOSE_POSITION, 1.0, 0.8, 1.0);

}
