package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.autonomous.AutoConstants.ParkingLocation.OBSERVATION_ZONE;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.commands.ArmCommands;
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
import org.firstinspires.ftc.teamcode.util.RobotGlobal;
import org.firstinspires.ftc.teamcode.util.commands.Commands;
import org.firstinspires.ftc.teamcode.vision.LLVision;

/**
 * Starts facing wall on tile X with edge on the center line
 * <p>
 * Scores high chamber, then collects and scores 3 neutral samples on high basket
 * <p>
 * Can park wherever
 */

// TODO: re-enable photon once it's fixed
//@Photon
@Config
@Disabled
@Autonomous(name = "New Left Side 3+1", group = "Auto")
public class NewAuto_Left_3_1 extends AutoTemplate {
    // Poses
    private final Pose startingPose = new Pose(8, 80, Math.toRadians(180));
    private final Pose scorePreloadPose = AutoConstants.CHAMBER_LEFT_SCORE_POSE;
    private final Pose collectOnePose = new Pose(24, 106, Math.toRadians(35));
    private final Pose scoreOnePose = AutoConstants.BASKET_SCORE_POSE;
    private final Pose collectTwoPose = new Pose(17, 128, Math.toRadians(0));
    private final Pose scoreTwoPose = AutoConstants.BASKET_SCORE_POSE;
    private final Pose collectThreePose = new Pose(19, 129, Math.toRadians(26));
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
    public static double preloadMaxSpeed = 0.7; // Speed reduction on the preload path
    public static long preloadPathDelay = 850; // Delay to allow for pivot to move before following first path
    public static long collectDelay = 400; // Delay in ms between extending and grabbing to allow for vision to align

    @Override
    public void buildPaths() {
        scorePreloadPath = new Path(new BezierLine(new Point(startingPose), new Point(scorePreloadPose)));
        scorePreloadPath.setConstantHeadingInterpolation(startingPose.getHeading());
        scorePreloadPath.setPathEndTimeoutConstraint(300);

        collectOnePath = new Path(new BezierLine(new Point(scorePreloadPose), new Point(collectOnePose)));
        collectOnePath.setLinearHeadingInterpolation(scorePreloadPose.getHeading(), collectOnePose.getHeading());

        scoreOnePath = new Path(new BezierLine(new Point(collectOnePose), new Point(scoreOnePose)));
        scoreOnePath.setLinearHeadingInterpolation(collectOnePose.getHeading(), scoreOnePose.getHeading());

        collectTwoPath = new Path(new BezierLine(new Point(scoreOnePose), new Point(collectTwoPose)));
        collectTwoPath.setLinearHeadingInterpolation(scoreOnePose.getHeading(), collectTwoPose.getHeading());

        scoreTwoPath = new Path(new BezierLine(new Point(collectTwoPose), new Point(scoreTwoPose)));
        scoreTwoPath.setLinearHeadingInterpolation(collectTwoPose.getHeading(), scoreTwoPose.getHeading());

        collectThreePath = new Path(new BezierLine(new Point(scoreTwoPose), new Point(collectThreePose)));
        collectThreePath.setLinearHeadingInterpolation(scoreTwoPose.getHeading(), collectThreePose.getHeading());

        scoreThreePath = new Path(new BezierLine(new Point(collectThreePose), new Point(scoreThreePose)));
        scoreThreePath.setLinearHeadingInterpolation(collectThreePose.getHeading(), scoreThreePose.getHeading());

        if (RobotGlobal.parkingLocation == OBSERVATION_ZONE) {
            parkPath = new Path(new BezierCurve(new Point(scoreThreePose), new Point(AutoConstants.OBVZONE_PARKING_POSE)));
            parkPath.setLinearHeadingInterpolation(scoreThreePose.getHeading(), AutoConstants.OBVZONE_PARKING_POSE.getHeading());
        } else {
            parkPath = new Path(new BezierCurve(new Point(scoreThreePose), new Point(60, 122, Point.CARTESIAN), new Point(AutoConstants.ASCENT_PARKING_POSE)));
            parkPath.setLinearHeadingInterpolation(scoreThreePose.getHeading(), AutoConstants.ASCENT_PARKING_POSE.getHeading());
        }
    }

    @Override
    protected void initSequence() {
        Claw.getInstance().setState(ClawConstants.REST_STATE);
        Claw.getInstance().periodic();
    }

    @Override
    protected Command makeAutoSequence() {
        return Commands.sequence(
                // Drive up to chamber and score
                Commands.parallel(
                        Commands.waitMillis(preloadPathDelay).andThen(new FollowPathCommand(scorePreloadPath, preloadMaxSpeed)),
                        Commands.defer(ArmCommands.STOW_TO_CHAMBER, Arm.getInstance())
                ),
                Commands.defer(ArmCommands.SCORE_SPECIMEN, Arm.getInstance()),
                // Drive to first sample and extend collector
                Commands.parallel(
                        new FollowPathCommand(collectOnePath),
                        Commands.defer(ArmCommands.CHAMBER_TO_STOW, Arm.getInstance())
                ),
                Commands.runOnce(() -> Arm.getInstance().setScoreType(Arm.ScoreType.SAMPLE)),
                Commands.defer(ArmCommands.STOW_TO_SAMPLE_COLLECT, Arm.getInstance()),
                Commands.waitMillis(collectDelay),
                // Collect sample
                Commands.defer(ArmCommands.COLLECT_SAMPLE, Claw.getInstance()),
                Commands.defer(ArmCommands.GRAB, Claw.getInstance()),
                Commands.defer(ArmCommands.SAMPLE_COLLECT_TO_STOW, Arm.getInstance()),
                // Go up to basket and score
                Commands.parallel(
                        Commands.defer(ArmCommands.STOW_TO_BASKET),
                        new FollowPathCommand(scoreOnePath)
                ),
                Commands.defer(ArmCommands.RELEASE, Claw.getInstance()),
                // Retract and go to collect second
                Commands.parallel(
                        Commands.defer(ArmCommands.BASKET_TO_STOW, Arm.getInstance()),
                        new FollowPathCommand(collectTwoPath)
                ),
                // Collect second sample
                Commands.defer(ArmCommands.STOW_TO_SAMPLE_COLLECT, Arm.getInstance()),
                Commands.waitMillis(collectDelay),
                Commands.defer(ArmCommands.COLLECT_SAMPLE, Claw.getInstance()),
                Commands.defer(ArmCommands.GRAB, Claw.getInstance()),
                Commands.defer(ArmCommands.SAMPLE_COLLECT_TO_STOW, Arm.getInstance()),
                // Score second sample
                Commands.parallel(
                        Commands.defer(ArmCommands.STOW_TO_BASKET, Arm.getInstance()),
                        new FollowPathCommand(scoreTwoPath)
                ),
                Commands.defer(ArmCommands.RELEASE, Claw.getInstance()),
                // Drive to final sample
                Commands.parallel(
                        Commands.defer(ArmCommands.BASKET_TO_STOW, Arm.getInstance()),
                        new FollowPathCommand(collectThreePath)
                ),
                Commands.defer(ArmCommands.STOW_TO_SAMPLE_COLLECT, Arm.getInstance()),
                Commands.waitMillis(collectDelay),
                // Collect third sample
                Commands.defer(ArmCommands.COLLECT_SAMPLE, Claw.getInstance()),
                Commands.defer(ArmCommands.GRAB, Arm.getInstance()),
                Commands.defer(ArmCommands.SAMPLE_COLLECT_TO_STOW, Arm.getInstance()),
                // Score third sample
                Commands.parallel(
                        Commands.defer(ArmCommands.STOW_TO_BASKET, Arm.getInstance()),
                        new FollowPathCommand(scoreThreePath)
                ),
                Commands.defer(ArmCommands.RELEASE, Claw.getInstance()),
                // Stow arm and go to park
                Commands.parallel(
                        Commands.defer(ArmCommands.BASKET_TO_STOW, Arm.getInstance()),
                        new FollowPathCommand(parkPath)
                )
        );
    }

    @Override
    protected Pose getStartingPose() {
        return startingPose;
    }
}
