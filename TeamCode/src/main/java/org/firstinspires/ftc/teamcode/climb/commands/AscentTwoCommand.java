package org.firstinspires.ftc.teamcode.climb.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotStatus;
import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.ArmConstants;
import org.firstinspires.ftc.teamcode.arm.commands.MovePivotCommand;
import org.firstinspires.ftc.teamcode.arm.commands.MoveSlideCommand;
import org.firstinspires.ftc.teamcode.claw.Claw;
import org.firstinspires.ftc.teamcode.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.climb.Climb;
import org.firstinspires.ftc.teamcode.climb.ClimbConstants;
import org.firstinspires.ftc.teamcode.util.commands.Commands;

@Config
public class AscentTwoCommand extends SequentialCommandGroup {
    private final Climb climb = Climb.getInstance();
    private final Arm arm = Arm.getInstance();
    private final ElapsedTime timer = new ElapsedTime();

    public static double CLIMB_PULL_OUT_AMOUNT = 600;
    public static double SLIDE_PULL_OUT_POWER = 0.25;

    public AscentTwoCommand() {
        addCommands(
                Commands.runOnce(() -> RobotStatus.setClimbState(RobotStatus.ClimbState.STARTED)),
                Commands.log("ClimbSequenceCommand","===============LATCHING==============="),
                Commands.runOnce(climb::readyHooks),
                // Latch climb onto slides
                Commands.runOnce(() -> climb.setPower(0.5)),
                Commands.waitUntil(() -> climb.getClimbPos() >= ClimbConstants.CLIMB_LATCH_POSITION),
                Commands.runOnce(() -> climb.setPower(0.0)),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_LATCH_POSITION),
                Commands.waitMillis(500),
                Commands.log("ClimbSequenceCommand","===============PULLING OUT==============="),
                // Make slides pull the climb out
                Commands.runOnce(() -> climb.setPower(0.5)),
                Commands.runOnce(timer::reset),
                Commands.runOnce(() -> arm.slideZeroing = true),
                Commands.runOnce(() -> arm.setSlidePower(SLIDE_PULL_OUT_POWER)),
                Commands.race(
                        Commands.waitUntil(() -> climb.getClimbPos() >= ClimbConstants.CLIMB_LATCH_POSITION + CLIMB_PULL_OUT_AMOUNT),
                        Commands.waitUntil(() -> timer.milliseconds() > 1000)
                ),
                Commands.runOnce(() -> climb.setPower(0)),
                Commands.runOnce(() -> arm.setSlidePower(0)),
                Commands.runOnce(() -> arm.setSlidePos(arm.getSlidePos())),
                Commands.runOnce(() -> arm.slideZeroing = false),
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
                Commands.runOnce(() -> Claw.getInstance().setState(ClawConstants.SAMPLE_COLLECTING_STATE)),
                Commands.runOnce(() -> RobotStatus.setClimbState(RobotStatus.ClimbState.ENGAGED)),
                // Confirm climb
                Commands.waitMillis(2000),
//                Commands.waitUntil(RobotStatus::isClimbReady),
//                Commands.run(() -> RobotStatus.setClimbState(RobotStatus.ClimbState.CLIMBING)),
                // Release slides
                new MoveSlideCommand(() -> ArmConstants.SLIDE_CLIMB_POSITION - 100),
                Commands.waitMillis(100),
                Commands.runOnce(() -> arm.setSlidePower(0)),
                Commands.runOnce(arm::floatNeutralMode),
                Commands.runOnce(() -> arm.slideZeroing = true),
                // Climb
                Commands.log("ClimbSequenceCommand","===============CLIMBING==============="),
                Commands.runOnce(() -> climb.setPower(1.0)),
                Commands.waitUntil(() -> climb.getClimbPos() >= ClimbConstants.CLIMB_IN_POSITION),
                new MovePivotCommand(() -> ArmConstants.PIVOT_CLIMBED_POSITION),
                Commands.runOnce(() -> climb.setPower(ClimbConstants.kF)),
                Commands.runOnce(climb::engageHooks),
                Commands.waitMillis(1000),
                Commands.runOnce(() -> climb.setPower(0)),
                Commands.runOnce(climb::servoDisable),
                Commands.run(() -> RobotStatus.setClimbState(RobotStatus.ClimbState.CLIMBED))
        );
        addRequirements(climb);
    }
}
