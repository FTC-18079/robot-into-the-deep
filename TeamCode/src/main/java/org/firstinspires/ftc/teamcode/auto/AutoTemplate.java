package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.RobotGlobal;

import static org.firstinspires.ftc.teamcode.auto.AutoConstants.ParkingPose.*;
import static org.firstinspires.ftc.teamcode.util.RobotGlobal.Alliance.*;

public class AutoTemplate extends LinearOpMode {
    boolean lastUp;
    boolean lastDown;
    boolean lastSquare;
    boolean lastCross;
    boolean lastCircle;

    @Override
    public void runOpMode() throws InterruptedException {
        while (opModeInInit() && !gamepad1.options) {
            config();
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
        if (checkInputs(gamepad1.circle, lastCircle)) RobotGlobal.parkingPose = RobotGlobal.parkingPose == SUBMERSIBLE ? SUBMERSIBLE : OBSERVATION_ZONE;

        telemetry.addData("Status", "Configuring Autonomous");
        telemetry.addData("Controls", "\nDelay: UP & DOWN \nToggle live view: CROSS \nSelect alliance: SQUARE \nParking pose: CIRCLE");
        telemetry.addLine();
        telemetry.addData("Selected auto delay", RobotGlobal.delayMs);
        telemetry.addData("Selected alliance", RobotGlobal.alliance);
        telemetry.addData("Selected parking spot", RobotGlobal.parkingPose);
        telemetry.addData("Live view on", RobotGlobal.liveView);
        telemetry.update();
    }

    public boolean checkInputs(boolean current, boolean last) {
        return last != current && current;
    }
}
