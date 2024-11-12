package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.auto.AutoConstants.ParkingPose.*;
import static org.firstinspires.ftc.teamcode.util.RobotGlobal.Alliance.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.chassis.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.claw.Claw;
import org.firstinspires.ftc.teamcode.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.util.RobotGlobal;
import org.firstinspires.ftc.teamcode.util.commands.Commands;

@Config
@Autonomous(name = "Meet 2 Auto", group = "Auto")
public class MeetTwoAuto extends LinearOpMode {
    RobotCore robot;

    boolean lastUp;
    boolean lastDown;
    boolean lastSquare;
    boolean lastCross;
    boolean lastCircle;

    // Poses
    private final Pose startingPose = new Pose(8, 80, Math.toRadians(180));
    private final Pose scorePreloadPose = new Pose(32, 80, Math.toRadians(180));
    private final Pose collectOnePose = new Pose(24, 106, Math.toRadians(35));
    private final Pose scoreOnePose = new Pose(14, 126, Math.toRadians(315));
    private final Pose collectTwoPose = new Pose();
    private final Pose scoreTwoPose = new Pose();
    private final Pose collectThreePose = new Pose();
    private final Pose scoreThreePose = new Pose();

    // Paths
    private Path scorePreloadPath;
    private Path collectOnePath;
    private Path scoreOnePath;
    private Path collectTwoPath;
    private Path scoreTwoPath;
    private Path collectThreePath;
    private Path scoreThreePath;

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
        Chassis.getInstance().setPosition(RobotGlobal.robotPose);

        // Schedule auto
        telemetry.addData("Status", "Scheduling commands");
        telemetry.update();
        if (RobotGlobal.alliance != NONE) robot.schedule(autoSequence()
                .andThen(new InstantCommand(Chassis.getInstance()::breakFollowing)));

        Claw.getInstance().setState(ClawConstants.REST_STATE);
        Claw.getInstance().periodic();

        while (opModeInInit()) {
            telemetry.addData("Status", "Initialized, Ready to start");
            telemetry.addData("Selected auto delay", RobotGlobal.delayMs);
            telemetry.addData("Live view on", RobotGlobal.liveView);
            telemetry.addData("Selected alliance", RobotGlobal.alliance);
            telemetry.addData("Selected parking spot", RobotGlobal.parkingPose);
            telemetry.update();

            // Sleep CPU a little
            sleep(50);
        }

        // Set a default alliance
        if (RobotGlobal.alliance == NONE) RobotGlobal.alliance = BLUE;

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
    }

    private Command autoSequence() {
        return Commands.sequence(
                Commands.waitMillis(RobotGlobal.delayMs),
                // Drive up to chamber and score
                Commands.parallel(
                        Commands.waitMillis(1200).andThen(new FollowPathCommand(scorePreloadPath)),
                        Commands.defer(ArmCommands.STOW_TO_CHAMBER, Arm.getInstance())
                ),
                Commands.defer(ArmCommands.SCORE_SPECIMEN, Arm.getInstance()),
                // Drive to first sample
                Commands.parallel(
                        new FollowPathCommand(collectOnePath),
                        Commands.defer(ArmCommands.CHAMBER_TO_STOW, Arm.getInstance())
                ),
                Commands.runOnce(() -> Arm.getInstance().setScoreType(Arm.ScoreType.SAMPLE)),
                // Collect
                Commands.defer(ArmCommands.STOW_TO_SAMPLE_COLLECT, Arm.getInstance()),
                Commands.waitMillis(700),
                Commands.sequence(
                        Commands.defer(ArmCommands.COLLECT_SAMPLE, Claw.getInstance()),
                        Commands.defer(ArmCommands.GRAB, Claw.getInstance()),
                        Commands.defer(ArmCommands.SAMPLE_COLLECT_TO_STOW, Arm.getInstance())
                ),
                Commands.waitMillis(500),
                // Go up to basket and score
                Commands.defer(ArmCommands.STOW_TO_BASKET),
                new FollowPathCommand(scoreOnePath),
                Commands.waitMillis(200),
                Commands.sequence(
                        Commands.defer(ArmCommands.RELEASE, Claw.getInstance()),
                        Commands.defer(ArmCommands.BASKET_TO_STOW, Claw.getInstance())
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
                    RobotGlobal.alliance = BLUE;
                    break;
                case BLUE:
                    RobotGlobal.alliance = RED;
                    break;
            }
        }
        // Toggle live view
        if (checkInputs(gamepad1.cross, lastCross)) RobotGlobal.liveView = !RobotGlobal.liveView;
        // Toggle parking pose
        if (checkInputs(gamepad1.circle, lastCircle)) RobotGlobal.parkingPose = RobotGlobal.parkingPose == ASCENT_ZONE ? OBSERVATION_ZONE : ASCENT_ZONE;

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
        telemetry.addData("Selected parking spot", RobotGlobal.parkingPose);
        telemetry.update();
    }

    private boolean checkInputs(boolean current, boolean last) {
        return (last != current) && current;
    }
}
