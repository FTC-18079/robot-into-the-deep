package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.vision.LLVision;

@TeleOp
public class LLTest extends OpMode {
    LLVision limelight;
    Servo pivot;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        RobotMap.getInstance().init(hardwareMap);
        limelight = new LLVision(telemetry);
        pivot = RobotMap.getInstance().PIVOT;
    }

    @Override
    public void loop() {
        limelight.periodic();
//        pivot.setPosition(limelight.getSampleAngle());
    }
}
