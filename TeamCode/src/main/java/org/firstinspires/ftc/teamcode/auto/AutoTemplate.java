package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.util.RobotGlobal;

import static org.firstinspires.ftc.teamcode.auto.AutoConstants.ParkingPose.*;
import static org.firstinspires.ftc.teamcode.util.RobotGlobal.Alliance.*;

public abstract class AutoTemplate extends LinearOpMode {
    boolean lastUp;
    boolean lastDown;
    boolean lastSquare;
    boolean lastCross;
    boolean lastCircle;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotGlobal.resetValues();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Configure auto variables
        while (opModeInInit() && !gamepad1.options) {
            config();
        }

        // Init hardware
        telemetry.addData("Status", "Initializing hardware");
        telemetry.update();
        RobotMap.getInstance().init(hardwareMap);

        // Create robot
        setStartPose();
        RobotCore robot = new RobotCore(
                RobotCore.OpModeType.EMPTY,
                telemetry,
                gamepad1,
                gamepad2
        );

        while (opModeInInit()) {
            telemetry.addData("Status", "Initialized, Ready to start");
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
        if (checkInputs(gamepad1.circle, lastCircle)) RobotGlobal.parkingPose = RobotGlobal.parkingPose == SUBMERSIBLE ? OBSERVATION_ZONE : SUBMERSIBLE;

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
}
