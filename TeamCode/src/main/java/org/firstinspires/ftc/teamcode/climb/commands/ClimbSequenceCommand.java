package org.firstinspires.ftc.teamcode.climb.commands;

import android.util.Log;

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
                Commands.log("ClimbSequenceCommand","===============LATCHING==============="),
                // Latch climb onto slides
                Commands.runOnce(() -> climb.setPower(-0.5)),
                Commands.waitUntil(() -> climb.getClimbPos() <= ClimbConstants.CLIMB_LATCH_POSITION),
                Commands.runOnce(() -> climb.setPower(0.0)),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_LATCH_POSITION),
                Commands.waitMillis(500),
                Commands.log("ClimbSequenceCommand","===============PULLING OUT==============="),
                // Make slides pull the climb out
                Commands.runOnce(() -> climb.setPower(0.5)),
                Commands.waitUntil(() -> climb.getClimbPos() >= ClimbConstants.CLIMB_LATCH_POSITION + 200),
                Commands.runOnce(() -> climb.setPower(0)),
                Commands.runOnce(climb::setFloat),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_PULL_CLIMB_POSITION),
                Commands.runOnce(climb::setBrake),
                Commands.waitMillis(500),
                // Unspool
                Commands.log("ClimbSequenceCommand","===============UNSPOOLING==============="),
                Commands.runOnce(() -> climb.setPower(-1.0)),
                Commands.waitUntil(() -> climb.getClimbPos() <= ClimbConstants.CLIMB_UNSPOOLED_POSITION),
                Commands.runOnce(() -> climb.setPower(0)),
                // Bring arm to engage
                Commands.log("ClimbSequenceCommand","===============ENGAGING==============="),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_ENGAGE_POSITION),
                new MovePivotCommand(() -> ArmConstants.PIVOT_SCORE_POSITION),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_CLIMB_POSITION),
                // Confirm climb
                Commands.waitMillis(5000),
                // Release slides
                Commands.runOnce(arm::floatNeutralMode),
                Commands.runOnce(() -> arm.slideZeroing = true),
                // Climb
                Commands.log("ClimbSequenceCommand","===============CLIMBING==============="),
                Commands.runOnce(() -> climb.setPower(1.0)),
                Commands.waitUntil(() -> climb.getClimbPos() >= ClimbConstants.CLIMB_IN_POSITION),
                Commands.runOnce(() -> climb.setPower(0))
        );
        addRequirements(climb);
    }
}
