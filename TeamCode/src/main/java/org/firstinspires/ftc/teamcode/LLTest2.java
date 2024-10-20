package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.collector.Collector;
import org.firstinspires.ftc.teamcode.util.hardware.SuccessServo;
import org.firstinspires.ftc.teamcode.vision.LLVision;

@TeleOp
public class LLTest2 extends OpMode {
    LLVision limelight;
    Collector collector;
    DcMotorEx motor;
    SuccessServo servo;

    @Override
    public void init() {
//        LLVision.resetInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        RobotMap.getInstance().init(hardwareMap);
        limelight = LLVision.getInstance();

        collector = Collector.getInstance();

        servo = new SuccessServo(RobotMap.getInstance().PIVOT);
    }

    @Override
    public void loop() {
//        limelight.periodic();
        limelight.updateResults();
        servo.setPosition(limelight.getServoPos());

        telemetry.addData("Motor pos", collector.getCurrentPosition());
        telemetry.addData("TY", limelight.getSampleTy());
        telemetry.update();
    }
}
