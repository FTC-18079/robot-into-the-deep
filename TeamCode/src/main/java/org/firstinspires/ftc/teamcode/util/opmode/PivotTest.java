package org.firstinspires.ftc.teamcode.util.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.arm.Pivot;

@Disabled
@TeleOp
public class PivotTest extends OpMode {
    private Pivot pivot;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        RobotMap.getInstance().init(hardwareMap);
        pivot = new Pivot();
    }

    @Override
    public void loop() {
        for (LynxModule m : RobotMap.getInstance().getLynxModules()) {
            m.clearBulkCache();
        }
        pivot.setPower(-gamepad1.left_stick_y);
        telemetry.addData("Joystick", -gamepad1.left_stick_y);
        telemetry.addData("Encoder pos", pivot.getPosition());
        telemetry.update();
    }
}
