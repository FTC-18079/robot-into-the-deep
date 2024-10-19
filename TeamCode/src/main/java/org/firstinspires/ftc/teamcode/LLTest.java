package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.vision.LLVision;

@TeleOp
public class LLTest extends OpMode {
    LLVision limelight;
    Servo pivot;

    double pos = 0.5;

    public static double[] HSV_RANGE_YELLOW = {
            13, 155, 8,    // LOW
            55, 255, 175    // HIGH
    };

    public static double[] HSV_RANGE_RED = {
            13, 155, 8,     // LOW
            55, 255, 175    // HIGH
    };

    public static double[] HSV_RANGE_BLUE = {
            104, 200, 0,    // LOW
            150, 255, 115   // HIGH
    };

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        RobotMap.getInstance().init(hardwareMap);
        RobotMap.getInstance().LIMELIGHT = hardwareMap.get(Limelight3A.class, "limelight");
        limelight = new LLVision(telemetry);
        limelight.setColorRange(HSV_RANGE_YELLOW);
        pivot = hardwareMap.get(Servo.class, "pivot");
    }

    @Override
    public void loop() {
        limelight.periodic();

        pos = (limelight.getSampleAngle() / 180.0);
        pivot.setPosition(pos);

        if (gamepad1.dpad_up) limelight.setPipeline(0);
        else if (gamepad1.dpad_down) limelight.setPipeline(4);

        if (gamepad1.cross) {
            limelight.setColorRange(HSV_RANGE_BLUE);
        } else if (gamepad1.circle) {
            limelight.setColorRange(HSV_RANGE_RED);
        } else if (gamepad1.square) {
            limelight.setColorRange(HSV_RANGE_YELLOW);
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
