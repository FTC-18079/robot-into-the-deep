package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.util.RobotGlobal;

@Autonomous(name = "Empty Autonomous", group = "Bad Autos")
public class EmptyAuto extends OpMode {
    @Override
    public void init() {
        RobotGlobal.robotPose = new Pose(0, 0, 0);
    }

    @Override
    public void loop() {

    }
}
