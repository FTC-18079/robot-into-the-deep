package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.util.RobotGlobal;

import static org.firstinspires.ftc.teamcode.autonomous.AutoConstants.ParkingLocation.*;
import static org.firstinspires.ftc.teamcode.util.RobotGlobal.Alliance.*;

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
        RobotGlobal.resetValues();

        // Configure auto variables
        while (opModeInInit() && !gamepad1.options) {
            config();
        }
        sleep(500);

        // Create robot
        RobotGlobal.robotPose = getStartingPose();
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
        if (RobotGlobal.alliance != NONE) robot.schedule(makeAutoSequence()
                .andThen(new InstantCommand(Chassis.getInstance()::breakFollowing)));

        // Move init servos
        initSequence();

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

        // Don't run anything without an alliance
        if (RobotGlobal.alliance == NONE) RobotGlobal.alliance = RobotGlobal.Alliance.BLUE;

        // Run robot
        while (opModeIsActive() && !isStopRequested()) {
            robot.run();
        }

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();
    }

    public void config() {
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

    public boolean checkInputs(boolean current, boolean last) {
        return (last != current) && current;
    }

    protected abstract Pose getStartingPose();

    protected abstract void initSequence();

    protected abstract void buildPaths();

    protected abstract Command makeAutoSequence();
}
