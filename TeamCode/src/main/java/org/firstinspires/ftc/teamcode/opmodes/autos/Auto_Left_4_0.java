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
import org.firstinspires.ftc.teamcode.arm.ArmConstants;
import org.firstinspires.ftc.teamcode.arm.commands.ArmCommands;
import org.firstinspires.ftc.teamcode.arm.commands.MovePivotCommand;
import org.firstinspires.ftc.teamcode.autonomous.AutoConstants;
import org.firstinspires.ftc.teamcode.autonomous.AutoTemplate;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.chassis.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.claw.Claw;
import org.firstinspires.ftc.teamcode.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.util.commands.Commands;
import org.firstinspires.ftc.teamcode.vision.LLVision;

/**
 * Starts facing opposite alliance on tile Y with edge on the center line
 * <p>
 * Scores high basket, then collects and scores 3 neutral samples on high basket
 * <p>
 * Parks in ascent zone
 */

@Config
@Autonomous(name = "Left Side 4+0", group = "Auto")
public class Auto_Left_4_0 extends AutoTemplate {
    // Poses
    private final Pose startingPose = new Pose(8, 104, Math.toRadians(0));
    private final Pose scorePreloadPose = AutoConstants.BASKET_SCORE_POSE;
    private final Pose collectOnePose = new Pose(17.25, 117.25, Math.toRadians(0));
    private final Pose scoreOnePose = AutoConstants.BASKET_SCORE_POSE;
    private final Pose collectTwoPose = new Pose(17, 126, Math.toRadians(0));
    private final Pose scoreTwoPose = AutoConstants.BASKET_SCORE_POSE;
    private final Pose collectThreePose = new Pose(18, 125, Math.toRadians(26));
    private final Pose scoreThreePose = AutoConstants.BASKET_SCORE_POSE;

    // Paths
    private PathChain scorePreloadPath, collectOnePath, scoreOnePath, collectTwoPath, scoreTwoPath, collectThreePath, scoreThreePath, parkPath;

    // Constants
    public static double preloadMaxSpeed = 0.8; // Speed reduction on the preload path
    public static long collectDelay = 150; // Delay in ms between extending and grabbing to allow for vision to align

    @Override
    public void buildPaths() {
        Follower follower = Chassis.getInstance().getFollower();

        scorePreloadPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startingPose), new Point(scorePreloadPose)))
                .setLinearHeadingInterpolation(startingPose.getHeading(), scorePreloadPose.getHeading())
                .build();

        collectOnePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePreloadPose), new Point(collectOnePose)))
                .setLinearHeadingInterpolation(scorePreloadPose.getHeading(), collectOnePose.getHeading())
                .build();

        scoreOnePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(collectOnePose), new Point(scoreOnePose)))
                .setLinearHeadingInterpolation(collectOnePose.getHeading(), scoreOnePose.getHeading())
                .build();

        collectTwoPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreOnePose), new Point(collectTwoPose)))
                .setLinearHeadingInterpolation(scoreOnePose.getHeading(), collectTwoPose.getHeading())
                .build();

        scoreTwoPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(collectTwoPose), new Point(scoreTwoPose)))
                .setLinearHeadingInterpolation(collectTwoPose.getHeading(), scoreTwoPose.getHeading())
                .build();

        collectThreePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreTwoPose), new Point(collectThreePose)))
                .setLinearHeadingInterpolation(scoreTwoPose.getHeading(), collectThreePose.getHeading())
                .build();

        scoreThreePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(collectThreePose), new Point(scoreThreePose)))
                .setLinearHeadingInterpolation(collectThreePose.getHeading(), scoreThreePose.getHeading())
                .build();

        parkPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scoreThreePose), new Point(60, 122, Point.CARTESIAN), new Point(AutoConstants.ASCENT_PARKING_POSE)))
                .setLinearHeadingInterpolation(scoreThreePose.getHeading(), AutoConstants.ASCENT_PARKING_POSE.getHeading())
                .build();
    }

    @Override
    protected Command makeAutoSequence() {
        return Commands.sequence(
                Commands.runOnce(() -> Arm.getInstance().setScoreType(Arm.ScoreType.SAMPLE)),
                Commands.runOnce(LLVision.getInstance()::setYellow),
                // Drive up to basket and score
                Commands.runOnce(() -> Claw.getInstance().setState(ClawConstants.REST_STATE)),
                Commands.parallel(
                        new FollowPathCommand(scorePreloadPath, true, preloadMaxSpeed),
                        Commands.defer(ArmCommands.STOW_TO_BASKET)
                ),
                Commands.waitMillis(150),
                Commands.defer(ArmCommands.RELEASE, Claw.getInstance()),
                // Drive to first sample and line up slides
                Commands.parallel(
                        new FollowPathCommand(collectOnePath),
                        Commands.defer(ArmCommands.BASKET_TO_STOW, Arm.getInstance())
                ),
                Commands.defer(ArmCommands.STOW_TO_SAMPLE_COLLECT, Arm.getInstance()),
                Commands.waitMillis(collectDelay),
                // Collect sample
                Commands.runOnce(() -> LLVision.getInstance().setClawOverride(0.95)),
                Commands.defer(ArmCommands.COLLECT_SAMPLE, Claw.getInstance()),
                Commands.defer(ArmCommands.GRAB, Claw.getInstance()),
                Commands.waitMillis(100),
                Commands.defer(ArmCommands.SAMPLE_COLLECT_TO_STOW, Arm.getInstance()),
                // Go up to basket and score
                Commands.parallel(
                        Commands.defer(ArmCommands.STOW_TO_BASKET),
                        new FollowPathCommand(scoreOnePath)
                ),
                Commands.waitMillis(150),
                Commands.defer(ArmCommands.RELEASE, Claw.getInstance()),
                // Retract and go to collect
                Commands.parallel(
                        Commands.defer(ArmCommands.BASKET_TO_STOW, Arm.getInstance()),
                        new FollowPathCommand(collectTwoPath)
                ),
                Commands.defer(ArmCommands.STOW_TO_SAMPLE_COLLECT, Arm.getInstance()),
                // Collect second sample
                Commands.runOnce(() -> LLVision.getInstance().setClawOverride(0.95)),
                Commands.waitMillis(collectDelay),
                Commands.defer(ArmCommands.COLLECT_SAMPLE, Claw.getInstance()),
                Commands.defer(ArmCommands.GRAB, Claw.getInstance()),
                Commands.waitMillis(100),
                Commands.defer(ArmCommands.SAMPLE_COLLECT_TO_STOW, Arm.getInstance()),
                // Go up to basket and score
                Commands.parallel(
                        Commands.defer(ArmCommands.STOW_TO_BASKET, Arm.getInstance()),
                        new FollowPathCommand(scoreTwoPath)
                ),
                Commands.waitMillis(150),
                Commands.defer(ArmCommands.RELEASE, Claw.getInstance()),
                // Retract and drive to final sample
                Commands.parallel(
                        Commands.defer(ArmCommands.BASKET_TO_STOW, Arm.getInstance()),
                        new FollowPathCommand(collectThreePath)
                ),
                Commands.defer(ArmCommands.STOW_TO_SAMPLE_COLLECT, Arm.getInstance()),
                // Collect third
                Commands.runOnce(() -> LLVision.getInstance().setClawOverride(1.0)),
//                Commands.runOnce(() -> LLVision.getInstance().setClawOverride(0.4)),
                Commands.waitMillis(collectDelay),
                Commands.defer(ArmCommands.COLLECT_SAMPLE, Claw.getInstance()),
                Commands.defer(ArmCommands.GRAB, Claw.getInstance()),
                Commands.waitMillis(100),
                Commands.defer(ArmCommands.SAMPLE_COLLECT_TO_STOW, Arm.getInstance()),
                // Basket and score,
                Commands.parallel(
                        Commands.defer(ArmCommands.STOW_TO_BASKET, Arm.getInstance()),
                        new FollowPathCommand(scoreThreePath)
                ),
                Commands.waitMillis(150),
                Commands.defer(ArmCommands.RELEASE, Claw.getInstance()),
                // Retract and park
                Commands.parallel(
                        Commands.defer(ArmCommands.BASKET_TO_STOW, Arm.getInstance()),
                        new FollowPathCommand(parkPath)
                ),
                // Touch bar
                Commands.runOnce(() -> Claw.getInstance().setState(ClawConstants.SAMPLE_SCORING_STATE)),
                new MovePivotCommand(() -> ArmConstants.PIVOT_SCORE_POSITION)
        );
    }

    @Override
    protected Pose getStartingPose() {
        return startingPose;
    }
}
