package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

@TeleOp(name = "Defense", group = "A")
public class DefenseOpMode extends OpMode {
    private Follower follower;
    private boolean isFieldCentric;
    private boolean lastCircle;
    private double m;

    private static final double DRIVE_SENSITIVITY = 1.1;
    private static final double ROTATIONAL_SENSITIVITY = 2.0;
    private static final double TRIGGER_DEADZONE = 0.1;
    private static final double ROTATION_DAMPEN = 0.9;

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.startTeleopDrive();
        isFieldCentric = true;
        lastCircle = false;
        m = 1;
    }

    @Override
    public void loop() {
        if (gamepad1.right_trigger > TRIGGER_DEADZONE) {
            m = 0.25;
        } else m = 1.0;

        if (gamepad1.circle && gamepad1.circle != lastCircle) {
            isFieldCentric = !isFieldCentric;
        }
        lastCircle = gamepad1.circle;

        if (gamepad1.triangle) {
            Pose oldPose = follower.getPose().copy();
            follower.setPose(new Pose(oldPose.getX(), oldPose.getY()));
        }

        follower.setTeleOpMovementVectors(
                m * -applyResponseCurve(gamepad1.left_stick_y, DRIVE_SENSITIVITY),
                m *-applyResponseCurve(gamepad1.left_stick_x, DRIVE_SENSITIVITY),
                m * -applyResponseCurve(gamepad1.right_stick_x, ROTATIONAL_SENSITIVITY) * ROTATION_DAMPEN,
                !isFieldCentric
        );
        follower.update();

        telemetry.addData("forward", -gamepad1.left_stick_y);
        telemetry.addData("lateral", -gamepad1.left_stick_x);
        telemetry.addData("rotation", -gamepad1.right_stick_x);
        telemetry.addData("field centric", isFieldCentric);
        telemetry.update();
    }

    private double applyResponseCurve(double value, double power) {
        return value * Math.pow(Math.abs(value), power - 1);
    }
}
