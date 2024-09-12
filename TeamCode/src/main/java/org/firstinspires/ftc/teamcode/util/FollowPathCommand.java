package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;

public class FollowPathCommand extends CommandBase {
    private final RobotCore robot;
    private final Path path;

    public FollowPathCommand(RobotCore robot, Path path) {
        this.robot = robot;
        this.path = path;
    }

    @Override
    public void initialize() {
        robot.followPath(path);
    }

    @Override
    public boolean isFinished() {
        return !robot.isBusy();
    }
}
