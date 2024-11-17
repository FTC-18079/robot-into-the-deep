package org.firstinspires.ftc.teamcode.arm.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.ArmConstants;

public class MoveSlideCommand extends CommandBase {
    private final Arm arm;
    private final double targetPos;
    private final ElapsedTime timer;

    public MoveSlideCommand(double targetPos) {
        this.arm = Arm.getInstance();
        this.targetPos = targetPos;
        timer = new ElapsedTime();
    }

    @Override
    public void initialize() {
        timer.reset();
        arm.setSlidePos(targetPos);
    }

    @Override
    public boolean isFinished() {
        return arm.slideAtSetPoint() || timer.milliseconds() > ArmConstants.SLIDE_TIMEOUT;
    }

    @Override
    public void end(boolean interrupted) {
        if (targetPos == ArmConstants.SLIDE_REST_POSITION && arm.slideAtSetPoint()) {
            arm.resetSlideEncoder();
        }
    }
}
