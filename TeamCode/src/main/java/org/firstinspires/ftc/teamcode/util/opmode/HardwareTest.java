package org.firstinspires.ftc.teamcode.util.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Hardware Test", group = "Util")
public class HardwareTest extends LinearOpMode {
    public static String MOTOR_NAME = "";
    public static boolean BRAKES_ENABLED = true;
    public static String SERVO_NAME = "";
    // RIGHT SERVO POS
    public static double SERVO_LEFT_POS = 0.0;
    // LEFT SERVO POS
    public static double SERVO_RIGHT_POS = 1.0;
    DcMotorEx motor;
    Servo servo;

    boolean motorExists = false;
    boolean servoExists = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        initHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Motor to test", MOTOR_NAME);
        telemetry.addData("Servo to test", SERVO_NAME);
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (motorExists) motor.setPower(-gamepad1.left_stick_y);

            telemetry.addData("LB", gamepad1.left_bumper);
            telemetry.addData("RB", gamepad1.right_bumper);
            if (motorExists) telemetry.addData("Motor position", motor.getCurrentPosition());
            if (motorExists) telemetry.addData("Motor power", motor.getPower());
            telemetry.addData("Motor velocity", motor.getVelocity());
            telemetry.update();

            if (gamepad1.left_bumper && servoExists) servo.setPosition(SERVO_LEFT_POS);
            if (gamepad1.right_bumper && servoExists) servo.setPosition(SERVO_RIGHT_POS);

            if (gamepad1.a) initHardware();
        }
    }

    public void initHardware() {
        if (!MOTOR_NAME.isEmpty()) {
            motorExists = true;
            motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
            motor.setZeroPowerBehavior(BRAKES_ENABLED ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else motorExists = false;

        if (!MOTOR_NAME.isEmpty()) {
            servoExists = true;
            servo = hardwareMap.get(Servo.class, SERVO_NAME);
        } else servoExists = false;
    }
}
