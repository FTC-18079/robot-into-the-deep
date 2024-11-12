package org.firstinspires.ftc.teamcode.chassis.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;

public class FollowPathCommand extends CommandBase {
    Telemetry telemetry;
    Chassis chassis;
    private final Path path;

    public FollowPathCommand(Path path) {
        telemetry = RobotCore.getTelemetry();
        chassis = Chassis.getInstance();
        this.path = path;

        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        chassis.followPath(path);
    }

    @Override
    public void execute() {
        telemetry.addLine();
        telemetry.addData("Path Running", chassis.isBusy());
    }

    @Override
    public boolean isFinished() {
        return !chassis.isBusy();
    }
}
