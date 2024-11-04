package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.arm.ArmConstants.PIVOT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class ArmTest extends OpMode {
    CRServo leftPivot;
    CRServo rightPivot;
    DcMotorEx encoder;
    PIDController pidController;
    public static double target = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        pidController = new PIDController(PIVOT.kP, PIVOT.kI, PIVOT.kD);
        encoder = hardwareMap.get(DcMotorEx.class, "rightBack");

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftPivot = hardwareMap.get(CRServo.class, "leftPivot");
        rightPivot = hardwareMap.get(CRServo.class, "rightPivot");

        leftPivot.setDirection(DcMotorSimple.Direction.FORWARD);
        rightPivot.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            pidController.setPID(PIVOT.kP, PIVOT.kI, PIVOT.kD);
        }

        if (gamepad1.x) {
            encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        pidController.setSetPoint(target);
        double output = pidController.calculate(encoder.getCurrentPosition());
        double ff = Math.cos(Math.toRadians(target / PIVOT.COUNTS_PER_REVOLUTION * 360.0)) * PIVOT.kF;

        leftPivot.setPower(output);
        rightPivot.setPower(output);

        telemetry.addData("Encoder Pos", encoder.getCurrentPosition());
        telemetry.addData("Encoder angle", encoder.getCurrentPosition() / PIVOT.COUNTS_PER_REVOLUTION * 360.0);
        telemetry.addData("Output", output);
        telemetry.addData("Target pos", target);
        telemetry.addData("Target angle", target / PIVOT.COUNTS_PER_REVOLUTION * 360.0);
        telemetry.update();
    }
}
