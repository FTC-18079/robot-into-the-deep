package org.firstinspires.ftc.teamcode.climb.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.ArmConstants;
import org.firstinspires.ftc.teamcode.arm.commands.MoveSlideCommand;
import org.firstinspires.ftc.teamcode.climb.Climb;
import org.firstinspires.ftc.teamcode.climb.ClimbConstants;
import org.firstinspires.ftc.teamcode.util.commands.Commands;

public class ClimbSequenceCommand extends SequentialCommandGroup {
    private final Climb climb = Climb.getInstance();
    private final Arm arm = Arm.getInstance();

    public ClimbSequenceCommand() {
        addCommands(
                Commands.runOnce(() -> climb.setClimbPos(ClimbConstants.CLIMB_LATCH_POSITION)),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_CLIMB_POSITION),
                Commands.waitMillis(100),
                Commands.runOnce(() -> arm.setPivotPos(ArmConstants.PIVOT_SCORE_POSITION))

        );
        addRequirements(climb);
    }
}
