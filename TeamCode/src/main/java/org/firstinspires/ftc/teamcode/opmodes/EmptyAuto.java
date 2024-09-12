package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoTemplate;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.util.RobotGlobal;

@Autonomous(name = "Empty Autonomous", group = "Bad Autos")
public class EmptyAuto extends AutoTemplate {
    @Override
    protected void setStartPose() {
        RobotGlobal.robotPose = new Pose(0, 0, 0);
    }
}
