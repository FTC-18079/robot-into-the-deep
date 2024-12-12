package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AutoTemplate;
import org.firstinspires.ftc.teamcode.chassis.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.util.commands.Commands;

import static org.firstinspires.ftc.teamcode.autonomous.AutoConstants.*;

/**
 * Starts facing wall on tile Y with edge on the center line
 * <p>
 * Scores high chamber, then pushes three samples into observation zones and starts cycling specimens
 * <p>
 * Parks in observation zone
 */

@Config
@Autonomous(name = "Right Side 0+4", group = "Auto")
public class Auto_Right_0_4 extends AutoTemplate {
    // Poses
    private final Pose startingPose = new Pose(8, 64, Math.toRadians(180));
    private final Pose scorePreloadPose = CHAMBER_RIGHT_SCORE_POSE;
    private final Pose behindOnePose = new Pose(62, 29, Math.toRadians(180));
    private final Pose pushOnePose = new Pose(16, 29, Math.toRadians(180));
    private final Pose behindTwoPose = new Pose(62, 19, Math.toRadians(180));
    private final Pose pushTwoPose = new Pose(16, 19, Math.toRadians(180));
    private final Pose behindThreePose = new Pose(62, 9, Math.toRadians(180));
    private final Pose pushThreePose = new Pose(WALL_COLLECT_X_POSITION, 9, Math.toRadians(180));
    private final Pose scoreOnePose = new Pose(CHAMBER_X_POSITION, CHAMBER_RIGHT_Y_POSITION - 2, Math.toRadians(180));
    private final Pose collectTwoPose = new Pose(WALL_COLLECT_X_POSITION, WALL_COLLECT_Y_POSITION, Math.toRadians(180));
    private final Pose scoreTwoPose = new Pose(CHAMBER_X_POSITION, CHAMBER_RIGHT_Y_POSITION - 4, Math.toRadians(180));
    private final Pose collectThreePose = collectTwoPose;
    private final Pose scoreThreePose = new Pose(CHAMBER_X_POSITION, CHAMBER_RIGHT_Y_POSITION - 6, Math.toRadians(180));
    private final Pose parkingPose = OBVZONE_PARKING_POSE;

    // Paths
    private Path scorePreloadPath;
    private Path behindOnePath;
    private Path pushOnePath;
    private Path behindTwoPath;
    private Path pushTwoPath;
    private Path behindThreePath;
    private Path pushThreePath;
    private Path scoreOnePath;
    private Path collectTwoPath;
    private Path scoreTwoPath;
    private Path collectThreePath;
    private Path scoreThreePath;
    private Path parkingPath;

    // Constants
    public static double preloadMaxSpeed = 0.65; // Speed reduction on the preload path
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

        behindOnePath = new Path(new BezierCurve(new Point(scorePreloadPose), new Point(27, 16, Point.CARTESIAN), new Point(64, 44, Point.CARTESIAN), new Point(behindOnePose)));
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

        behindThreePath = new Path(new BezierCurve(new Point(pushTwoPose), new Point(64, 18, Point.CARTESIAN), new Point(behindThreePose)));
        behindThreePath.setConstantHeadingInterpolation(behindThreePose.getHeading());
        behindThreePath.setPathEndTimeoutConstraint(0);

        pushThreePath = new Path(new BezierLine(new Point(behindThreePose), new Point(pushThreePose)));
        pushThreePath.setConstantHeadingInterpolation(pushThreePose.getHeading());

        scoreOnePath = new Path(new BezierLine(new Point(pushThreePose), new Point(scoreOnePose)));
        scoreOnePath.setConstantHeadingInterpolation(scoreOnePose.getHeading());

        collectTwoPath = new Path(new BezierLine(new Point(scoreOnePose), new Point(collectTwoPose)));
        collectTwoPath.setConstantHeadingInterpolation(collectTwoPose.getHeading());

        scoreTwoPath = new Path(new BezierLine(new Point(collectTwoPose), new Point(scoreTwoPose)));
        scoreTwoPath.setConstantHeadingInterpolation(scoreTwoPose.getHeading());

        collectThreePath = new Path(new BezierLine(new Point(scoreTwoPose), new Point(collectThreePose)));
        collectThreePath.setConstantHeadingInterpolation(collectThreePose.getHeading());

        scoreThreePath = new Path(new BezierLine(new Point(collectThreePose), new Point(scoreThreePose)));
        scoreThreePath.setConstantHeadingInterpolation(scoreThreePose.getHeading());

        parkingPath = new Path(new BezierLine(new Point(scoreThreePose), new Point(parkingPose)));
        parkingPath.setConstantHeadingInterpolation(parkingPose.getHeading());

    }

    @Override
    protected Command makeAutoSequence() {
        return Commands.sequence(
                // Drive up to high chamber and score
                new FollowPathCommand(scorePreloadPath, preloadMaxSpeed),
                new FollowPathCommand(behindOnePath),
                new FollowPathCommand(pushOnePath),
                new FollowPathCommand(behindTwoPath),
                new FollowPathCommand(pushTwoPath),
                new FollowPathCommand(behindThreePath),
                new FollowPathCommand(pushThreePath),
                new FollowPathCommand(scoreOnePath),
                new FollowPathCommand(collectTwoPath),
                new FollowPathCommand(scoreTwoPath),
                new FollowPathCommand(collectThreePath),
                new FollowPathCommand(scoreThreePath),
                new FollowPathCommand(parkingPath)
        );
    }
}
