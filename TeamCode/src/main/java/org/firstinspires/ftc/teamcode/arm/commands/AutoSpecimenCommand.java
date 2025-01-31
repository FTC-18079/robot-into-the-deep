package org.firstinspires.ftc.teamcode.arm.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.arm.Arm;

@Config
public class AutoSpecimenCommand extends CommandBase {
    private final Arm arm;
    private final ElapsedTime timer;

    public static double COMMAND_TIMEOUT = 500;
    public static double POWER = 0.5;

    public AutoSpecimenCommand() {
        arm = Arm.getInstance();
        timer = new ElapsedTime();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        timer.reset();
        arm.slideZeroing = true;
        arm.setSlidePower(POWER);
    }

    private boolean isStopped() {
        return timer.milliseconds() > 15 && arm.getSlideVelocity() <= 30;
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= COMMAND_TIMEOUT /* || isStopped() */;
    }

    @Override
    public void end(boolean interrupted) {
        arm.setSlidePower(0);
        arm.slideZeroing = false;
        arm.setSlidePos(arm.getSlidePos());
    }
}
