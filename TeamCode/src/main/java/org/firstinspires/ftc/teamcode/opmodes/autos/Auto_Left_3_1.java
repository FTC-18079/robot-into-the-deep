package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.autonomous.AutoConstants.ParkingLocation.*;
import static org.firstinspires.ftc.teamcode.util.RobotGlobal.Alliance;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.commands.ArmCommands;
import org.firstinspires.ftc.teamcode.autonomous.AutoConstants;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
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
@Autonomous(name = "Left Side 3+1", group = "Auto")
public class Auto_Left_3_1 extends LinearOpMode {
    RobotCore robot;

    boolean lastUp;
    boolean lastDown;
    boolean lastSquare;
    boolean lastCross;
    boolean lastCircle;

    // Poses
    private final Pose startingPose = new Pose(8, 80, Math.toRadians(180));
    private final Pose scorePreloadPose = AutoConstants.CHAMBER_LEFT_SCORE_POSE;
    private final Pose collectOnePose = new Pose(24, 106, Math.toRadians(35));
    private final Pose scoreOnePose = AutoConstants.BASKET_SCORE_POSE;
    private final Pose collectTwoPose = new Pose(13, 125, Math.toRadians(0));
    private final Pose scoreTwoPose = AutoConstants.BASKET_SCORE_POSE;
    private final Pose collectThreePose = new Pose(14, 124, Math.toRadians(26));
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

    @Override
    public void runOpMode() {
        // Init hardware
        telemetry.addData("Status", "Initializing hardware");
        telemetry.update();
        RobotMap.getInstance().init(hardwareMap);
        RobotGlobal.resetValues();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Configure auto variables
        while (opModeInInit() && !gamepad1.options) {
            config();
        }
        sleep(500);

        // Create robot
        sleep(100);
        RobotGlobal.robotPose = startingPose;
        buildPaths();
        robot = new RobotCore(
                RobotCore.OpModeType.AUTO,
                telemetry,
                gamepad1,
                gamepad2
        );

        // Schedule auto
        telemetry.addData("Status", "Scheduling commands");
        telemetry.update();
        if (RobotGlobal.alliance != Alliance.NONE) robot.schedule(autoSequence()
                .andThen(new InstantCommand(Chassis.getInstance()::breakFollowing)));

        Claw.getInstance().setState(ClawConstants.REST_STATE);
        Claw.getInstance().periodic();

        while (opModeInInit()) {
            telemetry.addData("Status", "Initialized, Ready to start");
            telemetry.addData("Selected auto delay", RobotGlobal.delayMs);
            telemetry.addData("Live view on", RobotGlobal.liveView);
            telemetry.addData("Selected alliance", RobotGlobal.alliance);
            telemetry.addData("Selected parking spot", RobotGlobal.parkingLocation);
            telemetry.update();

            // Sleep CPU a little
            sleep(50);
        }

        // Set a default alliance
        if (RobotGlobal.alliance == Alliance.NONE) RobotGlobal.alliance = Alliance.BLUE;

        // Run robot
        while (opModeIsActive() && !isStopRequested()) {
            robot.run();
        }

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();
    }

    private void buildPaths() {
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

    private Command autoSequence() {
        return Commands.sequence(
                Commands.runOnce(LLVision.getInstance()::setYellow),
                Commands.waitMillis(RobotGlobal.delayMs),
                // Drive up to chamber and score
                Commands.parallel(
                        Commands.waitMillis(1200).andThen(new FollowPathCommand(scorePreloadPath)),
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
                Commands.waitMillis(500),
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
                Commands.waitMillis(250),
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
                Commands.waitMillis(200),
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

    private void config() {
        // Add or remove delay
        if (checkInputs(gamepad1.dpad_up, lastUp)) RobotGlobal.delayMs += 100;
        if (checkInputs(gamepad1.dpad_down, lastDown) && RobotGlobal.delayMs > 0) RobotGlobal.delayMs -= 100;
        // Select alliance
        if (checkInputs(gamepad1.square, lastSquare)) {
            switch(RobotGlobal.alliance) {
                case NONE:
                case RED:
                    RobotGlobal.alliance = Alliance.BLUE;
                    break;
                case BLUE:
                    RobotGlobal.alliance = Alliance.RED;
                    break;
            }
        }
        // Toggle live view
        if (checkInputs(gamepad1.cross, lastCross)) RobotGlobal.liveView = !RobotGlobal.liveView;
        // Toggle parking pose
        if (checkInputs(gamepad1.circle, lastCircle)) RobotGlobal.parkingLocation = RobotGlobal.parkingLocation == ASCENT_ZONE ? OBSERVATION_ZONE : ASCENT_ZONE;

        // Set old inputs
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastSquare = gamepad1.square;
        lastCross = gamepad1.cross;
        lastCircle = gamepad1.circle;

        telemetry.addData("Status", "Configuring Autonomous");
        telemetry.addData("Controls", "\nDelay: UP & DOWN \nToggle live view: CROSS \nSelect alliance: SQUARE \nParking pose: CIRCLE");
        telemetry.addLine();
        telemetry.addData("Selected auto delay", RobotGlobal.delayMs);
        telemetry.addData("Live view on", RobotGlobal.liveView);
        telemetry.addData("Selected alliance", RobotGlobal.alliance);
        telemetry.addData("Selected parking spot", RobotGlobal.parkingLocation);
        telemetry.update();
    }

    private boolean checkInputs(boolean current, boolean last) {
        return (last != current) && current;
    }
}
