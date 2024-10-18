package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoTemplate;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.util.RobotGlobal;

@Photon
@Autonomous(name = "Empty Autonomous", group = "Bad Autos")
public class EmptyAuto extends AutoTemplate {
    @Override
    protected void setStartPose() {
        RobotGlobal.robotPose = new Pose(0, 0, 0);
    }

    @Override
    protected Command makeAutoSequence() {
        return new InstantCommand();
    }

    @Override
    protected void buildPaths() {

    }
}
