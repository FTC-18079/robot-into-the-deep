package org.firstinspires.ftc.teamcode.arm.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.arm.Arm;

@Config
public class AutoSpecimenCommand extends CommandBase {
    private final Arm arm;
    private final ElapsedTime timer;

    public static double VELOCITY_THRESHOLD = 15;
    public static double COMMAND_TIMEOUT = 600;

    public AutoSpecimenCommand() {
        arm = Arm.getInstance();
        timer = new ElapsedTime();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        timer.reset();
        arm.slideZeroing = true;
        arm.setSlidePower(0.3);
    }

    @Override
    public boolean isFinished() {
        return arm.getSlideVelocity() <= VELOCITY_THRESHOLD && timer.milliseconds() > 20 || timer.milliseconds() >= COMMAND_TIMEOUT;
    }

    @Override
    public void end(boolean interrupted) {
        arm.setSlidePower(0);
        arm.slideZeroing = false;
        arm.setSlidePos(arm.getSlidePos());
    }
}
