package org.firstinspires.ftc.teamcode.chassis.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hydra;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;

public class FollowPathCommand extends CommandBase {
    private final Telemetry telemetry;
    private final Chassis chassis;
    private final Path path;
    private final double maxPower;

    public FollowPathCommand(Path path) {
        this(path, 1.0);
    }

    public FollowPathCommand(Path path, double maxPower) {
        telemetry = Hydra.getInstance().getTelemetry();
        chassis = Chassis.getInstance();
        this.path = path;
        this.maxPower = maxPower;

        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        chassis.setMaxPower(maxPower);
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
