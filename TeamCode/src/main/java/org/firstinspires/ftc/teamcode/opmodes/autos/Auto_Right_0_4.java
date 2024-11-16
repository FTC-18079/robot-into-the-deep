package org.firstinspires.ftc.teamcode.opmodes.autos;

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

/**
 * Starts facing wall on tile Y with edge on the center line
 * <p>
 * Scores high chamber, then pushes three samples into observation zones and starts cycling specimens
 * <p>
 * Must park in observation zone
 */

@Disabled
@Config
@Autonomous(name = "Right Side 0+4")
public class Auto_Right_0_4 extends AutoTemplate {
    // Poses
    private final Pose startingPose = new Pose(8, 64, Math.toRadians(180));
    private final Pose scorePreloadPose = AutoConstants.CHAMBER_RIGHT_SCORE_POSE;
    private final Pose behindOnePose = new Pose(62, 29, Math.toRadians(180));
    private final Pose pushOnePose = new Pose();
    private final Pose behindTwoPose = new Pose();
    private final Pose pushTwoPose = new Pose();

    // Paths
    private Path scorePreloadPath;
    private Path behindOnePath;
    private Path pushOnePath;
    private Path behindTwoPath;
    private Path pushTwoPath;
    private Path behindThreePath;
    private Path pushThreePath;

    // Constants
    public static double preloadMaxSpeed = 0.7; // Speed reduction on the preload path
    public static long preloadPathDelay = 850; // Delay to allow for pivot to move before following first path
    public static long collectDelay = 400; // Delay in ms between extending and grabbing to allow for vision to align

    @Override
    protected Pose getStartingPose() {
        return startingPose;
    }

    @Override
    protected void initSequence() {
        Claw.getInstance().setState(ClawConstants.REST_STATE);
        Claw.getInstance().periodic();
    }

    @Override
    protected void buildPaths() {
        scorePreloadPath = new Path(new BezierLine(new Point(startingPose), new Point(scorePreloadPose)));
        scorePreloadPath.setConstantHeadingInterpolation(scorePreloadPose.getHeading());
        scorePreloadPath.setPathEndTimeoutConstraint(200);

        behindOnePath = new Path(new BezierCurve(new Point(scorePreloadPose), new Point(30, 20, Point.CARTESIAN), new Point(64, 42, Point.CARTESIAN), new Point(behindOnePose)));
        behindOnePath.setLinearHeadingInterpolation(scorePreloadPose.getHeading(), behindOnePose.getHeading());
    }

    @Override
    protected Command makeAutoSequence() {
        return Commands.sequence(
                // Drive up to high chamber and score
                Commands.parallel(
                        Commands.waitMillis(1200).andThen(new FollowPathCommand(scorePreloadPath)),
                        Commands.defer(ArmCommands.STOW_TO_CHAMBER, Arm.getInstance())
                ),
                Commands.defer(ArmCommands.SCORE_SPECIMEN, Arm.getInstance()),
                // Drive to sample and bring arm to stow
                Commands.parallel(
                        Commands.defer(ArmCommands.CHAMBER_TO_STOW, Arm.getInstance()),
                        new FollowPathCommand(behindOnePath)
                )
        );
    }
}
