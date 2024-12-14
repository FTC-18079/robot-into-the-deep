package org.firstinspires.ftc.teamcode.claw;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ClawConstants {
    public static long PIVOT_DELAY = 0;
    public static long GRAB_DELAY = 225;

    public static double CLAW_CLOSE_POSITION = 0.4;
    public static double CLAW_OPEN_POSITION = 1.0;

    public static double JOINT_ONE_EJECT_POSITION = 0.5;

    public static ClawState REST_STATE = new ClawState(CLAW_CLOSE_POSITION, 1.0, 0.95, 0.8);

    // SAMPLE POSITIONS
    public static double SAMPLE_COLLECT_JOINT_ONE_POS = 0;
    public static double SAMPLE_COLLECT_JOINT_TWO_POS = 1;

    public static ClawState SAMPLE_COLLECTING_STATE = new ClawState(CLAW_OPEN_POSITION, 1.0, 0.65, 0.0);
    public static ClawState SAMPLE_SCORING_STATE = new ClawState(CLAW_CLOSE_POSITION, 0, 0.55, 0.82);

    // SPECIMEN POSITIONS
    public static ClawState SPECIMEN_COLLECT_STATE = new ClawState(CLAW_OPEN_POSITION, 1.0, 0.42, 1.0);
    public static ClawState SPECIMEN_SCORING_STATE = new ClawState(CLAW_CLOSE_POSITION, 0.0, 0.65, 1.0);
    public static ClawState SPECIMEN_SCORE_STATE = new ClawState(CLAW_CLOSE_POSITION, 0.0, 0.75, 1.0);

}
