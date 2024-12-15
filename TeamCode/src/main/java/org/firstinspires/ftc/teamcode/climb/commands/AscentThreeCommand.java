package org.firstinspires.ftc.teamcode.climb.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.climb.Climb;

public class AscentThreeCommand extends SequentialCommandGroup {
    private final Climb climb = Climb.getInstance();
    private final Arm arm = Arm.getInstance();

    public AscentThreeCommand() {
        addCommands(

        );
        addRequirements(climb);
    }
}
