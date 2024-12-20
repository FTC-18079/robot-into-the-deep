package org.firstinspires.ftc.teamcode.util.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.arm.Arm;

@Config
@TeleOp(name = "Arm Test", group = "Tests")
public class ArmTest extends OpMode {
    Arm arm;
    public static double pivotTarget = 0;
    public static double slideTarget = 0;

    @Override
    public void init() {
        RobotMap.getInstance().init(hardwareMap);
        arm = Arm.getInstance();
        arm.onTeleopInit();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void init_loop() {
        for (LynxModule hub : RobotMap.getInstance().getLynxModules()) {
            hub.clearBulkCache();
        }
        telemetry.addData("Encoder Pos", arm.getPivotPos());
        telemetry.addData("Encoder angle", arm.getPivotPos());
        telemetry.addData("Target pos", arm.getPivotTarget());
        telemetry.addData("Target angle", arm.getPivotTarget());
        telemetry.update();
    }

    @Override
    public void loop() {
        for (LynxModule hub : RobotMap.getInstance().getLynxModules()) {
            hub.clearBulkCache();
        }

        if (gamepad1.a) {
            arm.updatePid();
        }

        arm.setPivotPower(-gamepad1.left_stick_y);

        telemetry.addData("Slide Pos", arm.getSlidePos());
        telemetry.addData("Pivot Pos", arm.getPivotPos());
        telemetry.update();
    }
}
