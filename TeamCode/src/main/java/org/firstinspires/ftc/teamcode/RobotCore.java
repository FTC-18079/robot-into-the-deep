package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotCore extends Robot {
    Telemetry telemetry;

    public RobotCore() {
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }
}
