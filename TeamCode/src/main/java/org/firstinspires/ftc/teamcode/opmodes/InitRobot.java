package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hydra;

@TeleOp(name = "Initialize Robot", group = "B")
public class InitRobot extends OpMode {
    @Override
    public void init() {
        Hydra.getInstance().robotInit();
    }

    @Override
    public void loop() {

    }
}
