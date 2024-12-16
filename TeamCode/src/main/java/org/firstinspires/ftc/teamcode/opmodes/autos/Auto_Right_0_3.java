package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotStatus;
import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.commands.ArmCommands;
import org.firstinspires.ftc.teamcode.autonomous.AutoTemplate;
import org.firstinspires.ftc.teamcode.chassis.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.claw.Claw;
import org.firstinspires.ftc.teamcode.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.util.commands.Commands;
import org.firstinspires.ftc.teamcode.vision.LLVision;

import static org.firstinspires.ftc.teamcode.autonomous.AutoConstants.*;

/**
 * Starts facing wall on tile Y with edge on the center line
 * <p>
 * Scores high chamber, then pushes three samples into observation zones and starts cycling specimens
 * <p>
 * Parks in observation zone
 */

@Config
@Autonomous(name = "Right Side 0+3", group = "Auto")
public class Auto_Right_0_3 extends AutoTemplate {
    // Poses
    private final Pose startingPose = new Pose(8, 64, Math.toRadians(180));
    private final Pose scorePreloadPose = CHAMBER_RIGHT_SCORE_POSE;
    private final Pose behindOnePose = new Pose(62, 29, Math.toRadians(180));
    private final Pose pushOnePose = new Pose(16, 29, Math.toRadians(180));
    private final Pose behindTwoPose = new Pose(62, 19, Math.toRadians(180));
    private final Pose pushTwoPose = new Pose(WALL_COLLECT_X_POSITION - 1, 19, Math.toRadians(180));
//    private final Pose behindThreePose = new Pose(62, 9, Math.toRadians(180));
//    private final Pose pushThreePose = new Pose(WALL_COLLECT_X_POSITION-1, 9, Math.toRadians(180));
    private final Pose scoreOnePose = new Pose(CHAMBER_X_POSITION, CHAMBER_RIGHT_Y_POSITION - 2.5, Math.toRadians(180));
    private final Pose collectTwoPose = new Pose(WALL_COLLECT_X_POSITION, WALL_COLLECT_Y_POSITION, Math.toRadians(180));
    private final Pose scoreTwoPose = new Pose(CHAMBER_X_POSITION, CHAMBER_RIGHT_Y_POSITION - 5, Math.toRadians(180));
//    private final Pose collectThreePose = collectTwoPose;
//    private final Pose scoreThreePose = new Pose(CHAMBER_X_POSITION, CHAMBER_RIGHT_Y_POSITION - 6, Math.toRadians(180));
    private final Pose parkingPose = OBVZONE_PARKING_POSE;
    private final Pose scoreControlPoint = new Pose(5.84415584,59.37662337662338, Math.toRadians(180));

    // Paths
    private Path scorePreloadPath;
    private Path behindOnePath;
    private Path pushOnePath;
    private Path behindTwoPath;
    private Path pushTwoPath;
//    private Path behindThreePath;
//    private Path pushThreePath;
    private Path scoreOnePath;
    private Path collectTwoPath;
    private Path scoreTwoPath;
    private Path collectThreePath;
    private Path scoreThreePath;
    private Path parkingPath;
    private Path collectOnePath;

    // Constants
    public static double preloadMaxSpeed = 0.5; // Speed reduction on the preload path
    public static long preloadPathDelay = 1000; // Delay to allow for pivot to move before following first path

    @Override
    protected Pose getStartingPose() {
        return startingPose;
    }

    @Override
    protected void buildPaths() {
        scorePreloadPath = new Path(new BezierLine(new Point(startingPose), new Point(scorePreloadPose)));
        scorePreloadPath.setConstantHeadingInterpolation(scorePreloadPose.getHeading());
        scorePreloadPath.setPathEndTimeoutConstraint(1000);

        behindOnePath = new Path(new BezierCurve(new Point(scorePreloadPose), new Point(27, 13, Point.CARTESIAN), new Point(64, 44, Point.CARTESIAN), new Point(behindOnePose)));
        behindOnePath.setConstantHeadingInterpolation(scorePreloadPose.getHeading());
        behindOnePath.setPathEndTimeoutConstraint(0);

        pushOnePath = new Path(new BezierLine(new Point(behindOnePose), new Point(pushOnePose)));
        pushOnePath.setConstantHeadingInterpolation(behindOnePose.getHeading());
        pushOnePath.setPathEndTimeoutConstraint(0);

        behindTwoPath = new Path(new BezierCurve(new Point(pushOnePose), new Point(64, 28, Point.CARTESIAN), new Point(behindTwoPose)));
        behindTwoPath.setConstantHeadingInterpolation(behindTwoPose.getHeading());
        behindTwoPath.setPathEndTimeoutConstraint(0);

        pushTwoPath = new Path(new BezierLine(new Point(behindTwoPose), new Point(pushTwoPose)));
        pushTwoPath.setConstantHeadingInterpolation(pushTwoPose.getHeading());
        pushTwoPath.setPathEndTimeoutConstraint(0);

//        behindThreePath = new Path(new BezierCurve(new Point(pushTwoPose), new Point(64, 18, Point.CARTESIAN), new Point(behindThreePose)));
//        behindThreePath.setConstantHeadingInterpolation(behindThreePose.getHeading());
//        behindThreePath.setPathEndTimeoutConstraint(0);

//        pushThreePath = new Path(new BezierLine(new Point(behindThreePose), new Point(pushThreePose)));
//        pushThreePath.setConstantHeadingInterpolation(pushThreePose.getHeading());

        collectOnePath = new Path(new BezierLine(new Point(pushTwoPose), new Point(collectTwoPose)));
        collectOnePath.setConstantHeadingInterpolation(collectTwoPose.getHeading());

        scoreOnePath = new Path(new BezierCurve(new Point(collectTwoPose), new Point(scoreControlPoint), new Point(scoreOnePose)));
        scoreOnePath.setConstantHeadingInterpolation(scoreOnePose.getHeading());

        collectTwoPath = new Path(new BezierLine(new Point(scoreOnePose), new Point(collectTwoPose)));
        collectTwoPath.setConstantHeadingInterpolation(collectTwoPose.getHeading());

        scoreTwoPath = new Path(new BezierCurve(new Point(collectTwoPose), new Point(scoreControlPoint) ,new Point(scoreTwoPose)));
        scoreTwoPath.setConstantHeadingInterpolation(scoreTwoPose.getHeading());

//        collectThreePath = new Path(new BezierLine(new Point(scoreTwoPose), new Point(collectThreePose)));
//        collectThreePath.setConstantHeadingInterpolation(collectThreePose.getHeading());
//
//        scoreThreePath = new Path(new BezierLine(new Point(collectThreePose), new Point(scoreThreePose)));
//        scoreThreePath.setConstantHeadingInterpolation(scoreThreePose.getHeading());

        parkingPath = new Path(new BezierLine(new Point(scoreTwoPose), new Point(parkingPose)));
        parkingPath.setConstantHeadingInterpolation(parkingPose.getHeading());

    }

    @Override
    protected Command makeAutoSequence() {
        return Commands.sequence(
                Commands.runOnce(() -> Arm.getInstance().setScoreType(Arm.ScoreType.SPECIMEN)),
                Commands.runOnce(() -> LLVision.getInstance().setRed()),
                Commands.runOnce(() -> Claw.getInstance().setState(ClawConstants.REST_STATE)),
                Commands.waitMillis(RobotStatus.delayMs),
                Commands.runOnce(() -> Claw.getInstance().setState(ClawConstants.REST_STATE)),
                Commands.waitMillis(20),
                Commands.parallel(
                        new FollowPathCommand(scorePreloadPath, preloadMaxSpeed),
                        Commands.defer(ArmCommands.STOW_TO_CHAMBER, Arm.getInstance())
                ),
                Commands.defer(ArmCommands.SCORE_SPECIMEN, Arm.getInstance()),
                Commands.parallel(
                        new FollowPathCommand(behindOnePath),
                        Commands.defer(ArmCommands.CHAMBER_TO_STOW, Arm.getInstance())
                ),
                // Push samples into zone
                new FollowPathCommand(pushOnePath),
                new FollowPathCommand(behindTwoPath),
                new FollowPathCommand(pushTwoPath),
//                new FollowPathCommand(behindThreePath),
//                new FollowPathCommand(pushThreePath),
                new FollowPathCommand(collectOnePath),
                // Collect first
                Commands.defer(ArmCommands.STOW_TO_SPECIMEN_COLLECT, Arm.getInstance()),
                Commands.waitMillis(500),
                Commands.defer(ArmCommands.GRAB, Arm.getInstance()),
                // Score
                Commands.parallel(
                        Commands.defer(ArmCommands.SPECIMEN_COLLECT_TO_CHAMBER, Arm.getInstance()),
                        new FollowPathCommand(scoreOnePath, 0.7)
                ),
                Commands.defer(ArmCommands.SCORE_SPECIMEN, Arm.getInstance()),
                // Collect second
                Commands.parallel(
                        new FollowPathCommand(collectTwoPath),
                        Commands.defer(ArmCommands.CHAMBER_TO_SPECIMEN_COLLECT, Arm.getInstance())
                ),
                Commands.waitMillis(500),
                Commands.defer(ArmCommands.GRAB, Arm.getInstance()),
                // Score
                Commands.parallel(
                        Commands.defer(ArmCommands.SPECIMEN_COLLECT_TO_CHAMBER, Arm.getInstance()),
                        new FollowPathCommand(scoreTwoPath, 0.7)
                ),
                Commands.defer(ArmCommands.SCORE_SPECIMEN, Arm.getInstance()),
                // Park
                Commands.parallel(
                        new FollowPathCommand(parkingPath),
                        Commands.defer(ArmCommands.CHAMBER_TO_STOW, Arm.getInstance())
                )
        );
    }
}
