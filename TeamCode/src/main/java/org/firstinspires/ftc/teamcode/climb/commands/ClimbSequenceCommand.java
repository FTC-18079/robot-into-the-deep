package org.firstinspires.ftc.teamcode.climb.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.climb.Climb;

public class ClimbSequenceCommand extends SequentialCommandGroup {
    private final Climb climb = Climb.getInstance();

    public ClimbSequenceCommand() {
        addCommands(

        );
        addRequirements(climb);
    }
}
