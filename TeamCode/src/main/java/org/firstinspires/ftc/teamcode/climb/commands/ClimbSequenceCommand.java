package org.firstinspires.ftc.teamcode.climb.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.ArmConstants;
import org.firstinspires.ftc.teamcode.arm.commands.MovePivotCommand;
import org.firstinspires.ftc.teamcode.arm.commands.MoveSlideCommand;
import org.firstinspires.ftc.teamcode.climb.Climb;
import org.firstinspires.ftc.teamcode.climb.ClimbConstants;
import org.firstinspires.ftc.teamcode.util.commands.Commands;

public class ClimbSequenceCommand extends SequentialCommandGroup {
    private final Climb climb = Climb.getInstance();
    private final Arm arm = Arm.getInstance();

    public ClimbSequenceCommand() {
        addCommands(
                Commands.parallel(
                        Commands.runOnce(() -> climb.setPower(0.25)),
                        new MoveSlideCommand(() -> ArmConstants.SLIDE_LATCH_POSITION)
                ),
                Commands.waitUntil(() -> climb.getClimbPos() >= ClimbConstants.CLIMB_LATCH_POSITION),
                Commands.runOnce(() -> climb.setPower(1.0)),
                Commands.waitMillis(100),
                new MovePivotCommand(() -> ArmConstants.PIVOT_SCORE_POSITION),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_CLIMB_POSITION),
                Commands.waitUntil(() -> climb.getClimbPos() >= ClimbConstants.CLIMB_UNSPOOLED_POSITION)
                        .andThen(Commands.waitMillis(100).andThen(Commands.runOnce(() -> climb.setPower(-1.0))))
        );
        addRequirements(climb);
    }
}
