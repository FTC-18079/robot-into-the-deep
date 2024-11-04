package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.arm.ArmConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.arm.Arm;

@Config
@TeleOp
public class ArmTest extends OpMode {
    Arm arm;
    public static double target = 0;

    @Override
    public void init() {
        RobotMap.getInstance().init(hardwareMap);
        arm = new Arm();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void init_loop() {
        for (LynxModule hub : RobotMap.getInstance().getLynxModules()) {
            hub.clearBulkCache();
        }
        telemetry.addData("Encoder Pos", arm.getPivotPos());
        telemetry.addData("Encoder angle", arm.getPivotPos() / PIVOT_COUNTS_PER_REVOLUTION * 360.0);
        telemetry.addData("Target pos", arm.getPivotTarget());
        telemetry.addData("Target angle", arm.getPivotTarget() / PIVOT_COUNTS_PER_REVOLUTION * 360.0);
        telemetry.update();
    }

    @Override
    public void loop() {
        for (LynxModule hub : RobotMap.getInstance().getLynxModules()) {
            hub.clearBulkCache();
        }
        arm.periodic();
        arm.setPivotPos(target);

        telemetry.addData("Encoder Pos", arm.getPivotPos());
        telemetry.addData("Encoder angle", arm.getPivotPos() / PIVOT_COUNTS_PER_REVOLUTION * 360.0);
        telemetry.addData("Target pos", arm.getPivotTarget());
        telemetry.addData("Target angle", arm.getPivotTarget() / PIVOT_COUNTS_PER_REVOLUTION * 360.0);
        telemetry.update();
    }

    @Override
    public void stop() {
        arm.setPivotOffset();
    }
}
