package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.auto.AutoConstants.checkAlliance;
import static org.firstinspires.ftc.teamcode.util.RobotGlobal.Alliance.RED;
import static org.firstinspires.ftc.teamcode.util.RobotGlobal.robotPose;
import static org.firstinspires.ftc.teamcode.util.RobotGlobal.alliance;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.auto.AutoConstants;
import org.firstinspires.ftc.teamcode.auto.AutoTemplate;
import org.firstinspires.ftc.teamcode.util.RobotGlobal;

public class OZ_1_3_Alliance extends AutoTemplate {


    @Override
    protected void setStartPose() {
        robotPose = AutoConstants.OBVZONE_STARTING_POSE;
        if (alliance == RED) robotPose = checkAlliance(robotPose);
        type = RobotCore.OpModeType.AUTO;
    }

    @Override
    protected Command makeAutoPath() {
        return new WaitCommand(RobotGlobal.delayMs);
    }
}
