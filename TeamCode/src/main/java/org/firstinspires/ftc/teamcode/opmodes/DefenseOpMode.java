package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@TeleOp(name = "Defense", group = "A")
public class DefenseOpMode extends OpMode {
    Follower follower;

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        follower.update();
    }
}
