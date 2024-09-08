package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.vision.ATVision;

@TeleOp(name = "Vision Test", group = "Tests")
public class VisionTest extends OpMode {
    ATVision atVision;

    @Override
    public void init() {
        RobotMap.getInstance().init(hardwareMap);
        atVision = new ATVision(true);
        FtcDashboard.getInstance().startCameraStream(atVision.livestream, 15);
    }

    @Override
    public void loop() {
        telemetry.addData("AprilTag FPS", atVision.getFPS());
    }
}
