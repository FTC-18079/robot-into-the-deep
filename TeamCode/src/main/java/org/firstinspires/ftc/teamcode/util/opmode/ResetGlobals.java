package org.firstinspires.ftc.teamcode.util.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.RobotGlobal;

@TeleOp(name = "Reset Robot Globals", group = "B")
public class ResetGlobals extends OpMode {
    @Override
    public void init() {
        telemetry.addLine("Press start to reset globals");
    }

    @Override
    public void start() {
        RobotGlobal.resetValues();
        requestOpModeStop();
    }

    @Override
    public void loop() {
    }
}
