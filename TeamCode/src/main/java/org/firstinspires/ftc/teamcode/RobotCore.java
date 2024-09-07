package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;

public class RobotCore extends Robot {

    public RobotCore() {

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
}
