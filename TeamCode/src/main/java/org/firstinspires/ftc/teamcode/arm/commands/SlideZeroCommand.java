package org.firstinspires.ftc.teamcode.arm.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.arm.Arm;

public class SlideZeroCommand extends CommandBase {
    private final Arm arm;
    private final ElapsedTime timer;

    public SlideZeroCommand() {
        arm = Arm.getInstance();
        timer = new ElapsedTime();
    }

    @Override
    public void initialize() {
        timer.reset();
        arm.slideZeroing = true;
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > 300;
    }

    @Override
    public void execute() {
        arm.setSlidePower(-0.5);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setSlidePower(0);
        arm.resetSlideEncoder();
        arm.setSlidePos(0);
        arm.slideZeroing = false;
    }
}
