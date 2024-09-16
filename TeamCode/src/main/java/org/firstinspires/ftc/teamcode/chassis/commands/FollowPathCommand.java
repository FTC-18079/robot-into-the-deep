package org.firstinspires.ftc.teamcode.chassis.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;

public class FollowPathCommand extends CommandBase {
    Chassis chassis;
    private final Path path;

    public FollowPathCommand(Path path) {
        chassis = Chassis.getInstance();
        this.path = path;

        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        chassis.followPath(path);
    }

    @Override
    public boolean isFinished() {
        return !chassis.isBusy();
    }
}
