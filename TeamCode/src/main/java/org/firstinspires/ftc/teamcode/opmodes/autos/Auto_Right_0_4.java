package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.commands.ArmCommands;
import org.firstinspires.ftc.teamcode.arm.commands.AutoSpecimenCommand;
import org.firstinspires.ftc.teamcode.autonomous.AutoTemplate;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.chassis.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.claw.Claw;
import org.firstinspires.ftc.teamcode.claw.ClawConstants;
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
@Autonomous(name = "Right Side 0+4 Spec", group = "Auto")
public class Auto_Right_0_4 extends AutoTemplate {
    private Follower follower;

    // Poses
    private final Pose startingPose = new Pose(8, 64, Math.toRadians(180));
    private final Pose scorePreloadPose = CHAMBER_RIGHT_SCORE_POSE.copy();
    private final Pose behindOnePose = new Pose(62, 29, Math.toRadians(180));
    private final Pose pushOnePose = new Pose(25, 29, Math.toRadians(180));
    private final Pose behindTwoPose = new Pose(62, 19, Math.toRadians(180));
    private final Pose pushTwoPose = new Pose(19, 19, Math.toRadians(180));
    private final Pose collectOnePose = new Pose(WALL_COLLECT_X_POSITION + 0.5, WALL_COLLECT_Y_POSITION, Math.toRadians(180));
    private final Pose scoreOnePose = new Pose(CHAMBER_X_POSITION, CHAMBER_RIGHT_Y_POSITION - 8, Math.toRadians(180));
    private final Pose collectTwoPose = new Pose(WALL_COLLECT_X_POSITION, WALL_COLLECT_Y_POSITION - 3, Math.toRadians(180));
    private final Pose scoreTwoPose = new Pose(CHAMBER_X_POSITION, CHAMBER_RIGHT_Y_POSITION - 7, Math.toRadians(180));
    private final Pose collectThreePose = new Pose(WALL_COLLECT_X_POSITION, WALL_COLLECT_Y_POSITION - 5, Math.toRadians(180));
    private final Pose scoreThreePose = new Pose(CHAMBER_X_POSITION - 0.75, CHAMBER_RIGHT_Y_POSITION - 6, Math.toRadians(180));
    private final Pose parkingPose = OBVZONE_PARKING_POSE.copy();
    private final Pose scoreControlPoint = new Pose(15,59.37662337662338, Math.toRadians(180));

    // Paths
    private PathChain scorePreloadPath;
    private PathChain pushPath;
    private PathChain scoreOnePath;
    private PathChain collectTwoPath;
    private PathChain scoreTwoPath;
    private PathChain collectThreePath;
    private PathChain scoreThreePath;
    private PathChain parkingPath;
    private PathChain collectOnePath;

    // Constants
    public static double preloadMaxSpeed = 0.9; // Speed reduction on the preload path
    public static double scoreSpeed = 0.85;
    public static double timeout = 250;
    public static long preloadPathDelay = 100; // Delay to allow for pivot to move before following first path
    public static long collectDelay = 425;

    @Override
    protected Pose getStartingPose() {
        return startingPose;
    }

    @Override
    protected void buildPaths() {
        follower = Chassis.getInstance().getFollower();

        scorePreloadPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startingPose), new Point(scorePreloadPose)))
                .setConstantHeadingInterpolation(scorePreloadPose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        pushPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePreloadPose), new Point(27, 13, Point.CARTESIAN), new Point(64, 44, Point.CARTESIAN), new Point(behindOnePose)))
                .setConstantHeadingInterpolation(scorePreloadPose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .addPath(new BezierLine(new Point(behindOnePose), new Point(pushOnePose)))
                .setConstantHeadingInterpolation(behindOnePose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .addPath(new BezierCurve(new Point(pushOnePose), new Point(64, 28, Point.CARTESIAN), new Point(behindTwoPose)))
                .setConstantHeadingInterpolation(behindTwoPose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .addPath(new BezierLine(new Point(behindTwoPose), new Point(pushTwoPose)))
                .setConstantHeadingInterpolation(pushTwoPose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        collectOnePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushTwoPose), new Point(collectOnePose)))
                .setConstantHeadingInterpolation(collectOnePose.getHeading())
                .build();

        scoreOnePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(collectOnePose), new Point(scoreControlPoint), new Point(scoreOnePose)))
                .setConstantHeadingInterpolation(scoreOnePose.getHeading())
                .setPathEndTimeoutConstraint(timeout)
                .build();

        collectTwoPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreOnePose), new Point(collectTwoPose)))
                .setConstantHeadingInterpolation(collectTwoPose.getHeading())
                .build();

        scoreTwoPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(collectTwoPose), new Point(scoreControlPoint) ,new Point(scoreTwoPose)))
                .setConstantHeadingInterpolation(scoreTwoPose.getHeading())
                .setPathEndTimeoutConstraint(timeout)
                .build();

        collectThreePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scoreTwoPose), new Point(collectThreePose)))
                .setConstantHeadingInterpolation(collectThreePose.getHeading())
                .build();

        scoreThreePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(collectThreePose), new Point(scoreControlPoint), new Point(scoreThreePose)))
                .setConstantHeadingInterpolation(scoreThreePose.getHeading())
                .setPathEndTimeoutConstraint(timeout)
                .build();

        parkingPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreThreePose), new Point(parkingPose)))
                .setConstantHeadingInterpolation(parkingPose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();
    }

    @Override
    protected Command makeAutoSequence() {
        return Commands.sequence(
                Commands.runOnce(() -> Arm.getInstance().setScoreType(Arm.ScoreType.SPECIMEN)),
                Commands.runOnce(() -> LLVision.getInstance().setRed()),
                Commands.runOnce(() -> Claw.getInstance().setState(ClawConstants.SPECIMEN_AUTO_SCORING_STATE)),
                Commands.parallel(
                        Commands.waitMillis(preloadPathDelay).andThen(new FollowPathCommand(scorePreloadPath, true, preloadMaxSpeed)),
                        Commands.defer(ArmCommands.STOW_TO_CHAMBER, Arm.getInstance())
                ),
                Commands.defer(ArmCommands.SCORE_SPECIMEN),
                Commands.parallel(
                        new FollowPathCommand(pushPath, false, 1).interruptOn(follower::isRobotStuck),
                        Commands.defer(ArmCommands.CHAMBER_TO_STOW)
                ),
                // Collect first
                new FollowPathCommand(collectOnePath),
                Commands.defer(ArmCommands.STOW_TO_SPECIMEN_COLLECT, Arm.getInstance()),
                Commands.waitMillis(collectDelay),
                new AutoSpecimenCommand(),
                Commands.defer(ArmCommands.GRAB, Arm.getInstance()),
                // Score
                Commands.parallel(
                        Commands.defer(ArmCommands.SPECIMEN_COLLECT_TO_CHAMBER, Arm.getInstance()),
                        new FollowPathCommand(scoreOnePath, true, scoreSpeed).interruptOn(follower::isRobotStuck)
                ),
                Commands.defer(ArmCommands.SCORE_SPECIMEN, Arm.getInstance()),
                // Collect second
                Commands.parallel(
                        new FollowPathCommand(collectTwoPath),
                        Commands.defer(ArmCommands.CHAMBER_TO_SPECIMEN_COLLECT, Arm.getInstance())
                ),
                Commands.waitMillis(collectDelay),
                new AutoSpecimenCommand(),
                Commands.defer(ArmCommands.GRAB, Arm.getInstance()),
                // Score
                Commands.parallel(
                        Commands.defer(ArmCommands.SPECIMEN_COLLECT_TO_CHAMBER, Arm.getInstance()),
                        new FollowPathCommand(scoreTwoPath, true, scoreSpeed).interruptOn(follower::isRobotStuck)
                ),
                Commands.defer(ArmCommands.SCORE_SPECIMEN, Arm.getInstance()),
                // Collect third
                Commands.parallel(
                        new FollowPathCommand(collectThreePath),
                        Commands.defer(ArmCommands.CHAMBER_TO_SPECIMEN_COLLECT, Arm.getInstance())
                ),
                Commands.waitMillis(collectDelay),
                new AutoSpecimenCommand(),
                Commands.defer(ArmCommands.GRAB, Arm.getInstance()),
                // Score
                Commands.parallel(
                        Commands.defer(ArmCommands.SPECIMEN_COLLECT_TO_CHAMBER, Arm.getInstance()),
                        new FollowPathCommand(scoreThreePath, true, scoreSpeed).interruptOn(follower::isRobotStuck)
                ),
                Commands.defer(ArmCommands.SCORE_SPECIMEN, Arm.getInstance()),
                // Park
//                Commands.parallel(
//                        new FollowPathCommand(parkingPath, false),
//                        Commands.defer(ArmCommands.CHAMBER_TO_STOW, Arm.getInstance())
//                )
                Commands.defer(ArmCommands.CHAMBER_TO_STOW, Arm.getInstance())
        );
    }
}
