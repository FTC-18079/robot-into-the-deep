package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.auto.AutoConstants.*;
import static org.firstinspires.ftc.teamcode.util.EasyPathBuilder.*;
import static org.firstinspires.ftc.teamcode.util.RobotGlobal.robotPose;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.auto.AutoTemplate;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.util.FollowPathCommand;
import org.firstinspires.ftc.teamcode.util.RobotGlobal;

@Autonomous(name = "OZ 1+3 Alliance Samples")
public class OZ_1_3_Alliance extends AutoTemplate {
    // Poses
    Pose scorePreloadPose = checkAlliance(new Pose(36, 60, Math.toRadians(0)));

    // Paths
    Path scorePreloadPath;

    @Override
    protected void setStartPose() {
        robotPose = checkAlliance(OBVZONE_STARTING_POSE);
        type = RobotCore.OpModeType.AUTO;
    }

    @Override
    public void buildPaths() {
        scorePreloadPath = constantHeadingPath(robotPose, scorePreloadPose, robotPose.getHeading());
    }

    @Override
    protected Command makeAutoSequence() {
        return new WaitCommand(RobotGlobal.delayMs)
                .andThen(new FollowPathCommand(robot, scorePreloadPath));
    }
}
