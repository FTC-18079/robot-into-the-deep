package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Drawing;
import org.firstinspires.ftc.teamcode.util.RobotGlobal;

@TeleOp(name = "TeleOp", group = "AAA")
public class TeleOpMode extends OpMode {
    RobotCore robot;

    @Override
    public void init() {
        RobotMap.getInstance().init(hardwareMap);
        Chassis.resetInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new RobotCore(RobotCore.OpModeType.TELEOP, telemetry, gamepad1, gamepad2);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Alliance", RobotGlobal.alliance);
        telemetry.addData("AprilTag FPS", robot.getFPS());
        telemetry.addData("Status", "Robot initialized, ready to enable");
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.run();
    }
}
