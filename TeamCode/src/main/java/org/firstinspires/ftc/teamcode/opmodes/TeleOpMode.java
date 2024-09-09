package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Drawing;
import org.firstinspires.ftc.teamcode.util.RobotGlobal;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOpMode extends OpMode {
    RobotCore robot;

    @Override
    public void init() {
        RobotMap.getInstance().init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        RobotCore robot = new RobotCore(RobotCore.OpModeType.TELEOP, telemetry, gamepad1, gamepad2);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Alliance", RobotGlobal.alliance);
        telemetry.addData("AprilTag FPS", robot.getFPS());
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.run();

        // Draw robot
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#4CAF50");
        Drawing.drawRobotOnCanvas(packet.fieldOverlay(), robot.getPoseEstimate());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
