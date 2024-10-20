package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.vision.LLVision;

@TeleOp
public class LLTest2 extends OpMode {
    LLVision limelight;
    Servo servo;

    @Override
    public void init() {
//        LLVision.resetInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        RobotMap.getInstance().init(hardwareMap);
        limelight = LLVision.getInstance();
        servo = RobotMap.getInstance().PIVOT;
    }

    @Override
    public void loop() {
        limelight.periodic();
        servo.setPosition(limelight.getServoPos());

        telemetry.addData("is running", limelight.isRunning());
        telemetry.addData("Servo Angle", limelight.getServoPos());
        telemetry.addData("Sample Angle", limelight.getSampleAngle());
        telemetry.addData("Result null?", limelight.getResult() == null);
        telemetry.addData("Color Result exists?", limelight.colorResultExists());
        telemetry.addData("Result valid?", limelight.getResult().isValid());
        telemetry.update();
    }
}
