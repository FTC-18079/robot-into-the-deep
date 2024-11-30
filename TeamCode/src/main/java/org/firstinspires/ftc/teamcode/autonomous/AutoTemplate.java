package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.RobotStatus;
import org.firstinspires.ftc.teamcode.util.commands.Commands;

import static org.firstinspires.ftc.teamcode.autonomous.AutoConstants.ParkingLocation.*;
import static org.firstinspires.ftc.teamcode.RobotStatus.Alliance.*;

public abstract class AutoTemplate extends LinearOpMode {
    protected RobotCore robot;
    protected Pose startingPose;

    boolean lastUp;
    boolean lastDown;
    boolean lastSquare;
    boolean lastCross;
    boolean lastCircle;

    @Override
    public void runOpMode() throws InterruptedException {
        // Init hardware
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initializing hardware");
        telemetry.update();

        RobotMap.getInstance().init(hardwareMap);
        RobotStatus.resetValues();

        // Configure auto variables
        while (opModeInInit() && !gamepad1.options) {
            config();
        }
        sleep(500);

        // Create robot
        RobotStatus.robotPose = getStartingPose();
        sleep(100);
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
        if (RobotStatus.alliance != NONE) robot.schedule(
                Commands.waitMillis(RobotStatus.delayMs)
                .andThen(makeAutoSequence())
                .andThen(Commands.runOnce(Chassis.getInstance()::breakFollowing))
        );

        // Move init servos
        initSequence();

        while (opModeInInit()) {
            telemetry.addData("Status", "Initialized, Ready to start");
            telemetry.addData("Selected auto delay", RobotStatus.delayMs);
            telemetry.addData("Live view on", RobotStatus.liveView);
            telemetry.addData("Selected alliance", RobotStatus.alliance);
            telemetry.addData("Selected parking spot", RobotStatus.parkingLocation);
            telemetry.update();

            // Sleep CPU a little
            sleep(50);
        }

        // Don't run anything without an alliance
        if (RobotStatus.alliance == NONE) RobotStatus.alliance = RobotStatus.Alliance.BLUE;

        // Run robot
        while (opModeIsActive() && !isStopRequested()) {
            robot.run();
        }

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();
    }

    public void config() {
        // Add or remove delay
        if (checkInputs(gamepad1.dpad_up, lastUp)) RobotStatus.delayMs += 100;
        if (checkInputs(gamepad1.dpad_down, lastDown) && RobotStatus.delayMs > 0) RobotStatus.delayMs -= 100;
        // Select alliance
        if (checkInputs(gamepad1.square, lastSquare)) {
            switch(RobotStatus.alliance) {
                case NONE:
                case RED:
                    RobotStatus.alliance = BLUE;
                    break;
                case BLUE:
                    RobotStatus.alliance = RED;
                    break;
            }
        }
        // Toggle live view
        if (checkInputs(gamepad1.cross, lastCross)) RobotStatus.liveView = !RobotStatus.liveView;
        // Toggle parking pose
        if (checkInputs(gamepad1.circle, lastCircle)) RobotStatus.parkingLocation = RobotStatus.parkingLocation == ASCENT_ZONE ? OBSERVATION_ZONE : ASCENT_ZONE;

        // Set old inputs
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastSquare = gamepad1.square;
        lastCross = gamepad1.cross;
        lastCircle = gamepad1.circle;

        telemetry.addData("Status", "Configuring Autonomous");
        telemetry.addData("Controls", "\nDelay: UP & DOWN \nToggle live view: CROSS \nSelect alliance: SQUARE \nParking pose: CIRCLE");
        telemetry.addLine();
        telemetry.addData("Selected auto delay", RobotStatus.delayMs);
        telemetry.addData("Live view on", RobotStatus.liveView);
        telemetry.addData("Selected alliance", RobotStatus.alliance);
        telemetry.addData("Selected parking spot", RobotStatus.parkingLocation);
        telemetry.update();
    }

    public boolean checkInputs(boolean current, boolean last) {
        return (last != current) && current;
    }

    protected abstract Pose getStartingPose();

    protected abstract void initSequence();

    protected abstract void buildPaths();

    protected abstract Command makeAutoSequence();
}
