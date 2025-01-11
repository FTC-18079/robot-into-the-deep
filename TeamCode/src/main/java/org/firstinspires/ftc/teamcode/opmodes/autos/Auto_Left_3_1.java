package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.ArmConstants;
import org.firstinspires.ftc.teamcode.arm.commands.ArmCommands;
import org.firstinspires.ftc.teamcode.arm.commands.MovePivotCommand;
import org.firstinspires.ftc.teamcode.arm.commands.MoveSlideCommand;
import org.firstinspires.ftc.teamcode.autonomous.AutoConstants;
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
import org.firstinspires.ftc.teamcode.vision.VisionConstants;

/**
 * Starts facing wall on tile X with edge on the center line
 * <p>
 * Scores high chamber, then collects and scores 3 neutral samples on high basket
 * <p>
 * Parks in ascent zone
 */

@Disabled
@Config
@Autonomous(name = "Left Side 3+1", group = "Auto")
public class Auto_Left_3_1 extends AutoTemplate {
    // Poses
    private final Pose startingPose = new Pose(8, 80, Math.toRadians(180));
    private final Pose scorePreloadPose = AutoConstants.CHAMBER_LEFT_SCORE_POSE;
    private final Pose collectOnePose = new Pose(17.5, 117.25, Math.toRadians(0));
    private final Pose scoreOnePose = AutoConstants.BASKET_SCORE_POSE;
    private final Pose collectTwoPose = new Pose(17, 126, Math.toRadians(0));
    private final Pose scoreTwoPose = AutoConstants.BASKET_SCORE_POSE;
    private final Pose collectThreePose = new Pose(20, 125, Math.toRadians(27));
    private final Pose scoreThreePose = AutoConstants.BASKET_SCORE_POSE;

    // Paths
    private Path scorePreloadPath;
    private Path collectOnePath;
    private Path scoreOnePath;
    private Path collectTwoPath;
    private Path scoreTwoPath;
    private Path collectThreePath;
    private Path scoreThreePath;
    private Path parkPath;

    // Constants
    public static double preloadMaxSpeed = 0.4; // Speed reduction on the preload path
    public static long collectDelay = 125; // Delay in ms between extending and grabbing to allow for vision to align

    @Override
    public void buildPaths() {
        scorePreloadPath = new Path(new BezierLine(new Point(startingPose), new Point(scorePreloadPose)));
        scorePreloadPath.setConstantHeadingInterpolation(startingPose.getHeading());
        scorePreloadPath.setPathEndTimeoutConstraint(800);

        collectOnePath = new Path(new BezierLine(new Point(scorePreloadPose), new Point(collectOnePose)));
        collectOnePath.setLinearHeadingInterpolation(scorePreloadPose.getHeading(), collectOnePose.getHeading());

        scoreOnePath = new Path(new BezierLine(new Point(collectOnePose), new Point(scoreOnePose)));
        scoreOnePath.setLinearHeadingInterpolation(collectOnePose.getHeading(), scoreOnePose.getHeading());
        scoreOnePath.setPathEndTimeoutConstraint(800);

        collectTwoPath = new Path(new BezierLine(new Point(scoreOnePose), new Point(collectTwoPose)));
        collectTwoPath.setLinearHeadingInterpolation(scoreOnePose.getHeading(), collectTwoPose.getHeading());

        scoreTwoPath = new Path(new BezierLine(new Point(collectTwoPose), new Point(scoreTwoPose)));
        scoreTwoPath.setLinearHeadingInterpolation(collectTwoPose.getHeading(), scoreTwoPose.getHeading());
        scoreTwoPath.setPathEndTimeoutConstraint(800);

        collectThreePath = new Path(new BezierLine(new Point(scoreTwoPose), new Point(collectThreePose)));
        collectThreePath.setLinearHeadingInterpolation(scoreTwoPose.getHeading(), collectThreePose.getHeading());

        scoreThreePath = new Path(new BezierLine(new Point(collectThreePose), new Point(scoreThreePose)));
        scoreThreePath.setLinearHeadingInterpolation(collectThreePose.getHeading(), scoreThreePose.getHeading());
        scoreThreePath.setPathEndTimeoutConstraint(800);

        parkPath = new Path(new BezierCurve(new Point(scoreThreePose), new Point(60, 122, Point.CARTESIAN), new Point(AutoConstants.ASCENT_PARKING_POSE)));
        parkPath.setLinearHeadingInterpolation(scoreThreePose.getHeading(), AutoConstants.ASCENT_PARKING_POSE.getHeading());
        parkPath.setPathEndTimeoutConstraint(0);
    }

    @Override
    protected Command makeAutoSequence() {
        return Commands.sequence(
                Commands.runOnce(() -> Arm.getInstance().setScoreType(Arm.ScoreType.SPECIMEN)),
                Commands.runOnce(LLVision.getInstance()::setYellow),
                // Drive up to chamber and score
                Commands.runOnce(() -> Claw.getInstance().setState(ClawConstants.REST_STATE)),
                Commands.parallel(
                        new FollowPathCommand(scorePreloadPath, preloadMaxSpeed),
                        Commands.defer(ArmCommands.STOW_TO_CHAMBER, Arm.getInstance())
                ),
                Commands.defer(ArmCommands.SCORE_SPECIMEN, Arm.getInstance()),
                // Drive to first sample and line up slides
                Commands.parallel(
                        new FollowPathCommand(collectOnePath),
                        Commands.defer(ArmCommands.CHAMBER_TO_STOW, Arm.getInstance())
                ),
                Commands.runOnce(() -> Arm.getInstance().setScoreType(Arm.ScoreType.SAMPLE)),
                Commands.defer(ArmCommands.STOW_TO_SAMPLE_COLLECT, Arm.getInstance()),
                Commands.waitMillis(collectDelay),
                // Collect sample
                Commands.runOnce(() -> LLVision.getInstance().setClawOverride(1)),
                Commands.defer(ArmCommands.COLLECT_SAMPLE, Claw.getInstance()),
                Commands.defer(ArmCommands.GRAB, Claw.getInstance()),
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
                        Commands.waitMillis(100).andThen(Commands.defer(ArmCommands.BASKET_TO_STOW, Arm.getInstance())),
                        new FollowPathCommand(collectTwoPath)
                ),
                Commands.defer(ArmCommands.STOW_TO_SAMPLE_COLLECT, Arm.getInstance()),
                // Collect second sample
                Commands.runOnce(() -> LLVision.getInstance().setClawOverride(1)),
                Commands.waitMillis(collectDelay),
                Commands.defer(ArmCommands.COLLECT_SAMPLE, Claw.getInstance()),
                Commands.defer(ArmCommands.GRAB, Claw.getInstance()),
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
                        Commands.waitMillis(100).andThen(Commands.defer(ArmCommands.BASKET_TO_STOW, Arm.getInstance())),
                        new FollowPathCommand(collectThreePath)
                ),
                Commands.defer(ArmCommands.STOW_TO_SAMPLE_COLLECT, Arm.getInstance()),
                // Collect third
                Commands.runOnce(() -> LLVision.getInstance().setClawOverride(0.4)),
                Commands.waitMillis(collectDelay),
                Commands.defer(ArmCommands.COLLECT_SAMPLE, Claw.getInstance()),
                Commands.defer(ArmCommands.GRAB, Claw.getInstance()),
                Commands.defer(ArmCommands.SAMPLE_COLLECT_TO_STOW, Arm.getInstance()),
                // Basket and score,
                Commands.runOnce(LLVision.getInstance()::disableClawOverride),
                Commands.parallel(
                        Commands.defer(ArmCommands.STOW_TO_BASKET, Arm.getInstance()),
                        new FollowPathCommand(scoreThreePath)
                ),
                Commands.waitMillis(150),
                Commands.defer(ArmCommands.RELEASE, Claw.getInstance()),
                // Retract and park
                Commands.parallel(
                        Commands.sequence(
                                Commands.waitMillis(200),
                                Commands.defer(ArmCommands.BASKET_TO_STOW, Arm.getInstance())
                        ),
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
