package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.util.RobotGlobal;

import static org.firstinspires.ftc.teamcode.auto.AutoConstants.ParkingPose.*;
import static org.firstinspires.ftc.teamcode.util.RobotGlobal.Alliance.*;

public abstract class AutoTemplate extends LinearOpMode {
    public RobotCore.OpModeType type = RobotCore.OpModeType.EMPTY;
    protected RobotCore robot;

    boolean lastUp;
    boolean lastDown;
    boolean lastSquare;
    boolean lastCross;
    boolean lastCircle;

    @Override
    public void runOpMode() throws InterruptedException {
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

        // Create robot
        rotatePoses();
        setStartPose();
        buildPaths();
        robot = new RobotCore(
                type,
                telemetry,
                gamepad1,
                gamepad2
        );
        Chassis.getInstance().setPosition(RobotGlobal.robotPose);

        // Schedule auto
        telemetry.addData("Status", "Scheduling commands");
        telemetry.update();
        if (RobotGlobal.alliance != NONE) robot.schedule(makeAutoSequence()
                .andThen(new InstantCommand(Chassis.getInstance()::breakFollowing)));

        initSequence();

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

        // Don't run anything without an alliance
        if (RobotGlobal.alliance == NONE) requestOpModeStop();

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

    public boolean checkInputs(boolean current, boolean last) {
        return (last != current) && current;
    }

    // Method must be overwritten to set robot starting pose
    protected abstract void setStartPose();

    protected abstract Command makeAutoSequence();

    protected abstract void buildPaths();

    protected abstract void initSequence();

    protected abstract void rotatePoses();
}
