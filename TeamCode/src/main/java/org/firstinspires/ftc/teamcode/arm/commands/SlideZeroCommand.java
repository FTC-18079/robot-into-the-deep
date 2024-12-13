package org.firstinspires.ftc.teamcode.arm.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.ArmConstants;
import org.firstinspires.ftc.teamcode.util.commands.Commands;

public class SlideZeroCommand extends CommandBase {
    private final Arm arm;
    private final ElapsedTime timer;

    public SlideZeroCommand() {
        arm = Arm.getInstance();
        timer = new ElapsedTime();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        timer.reset();
        arm.slideZeroing = true;
        Log.i("SlideZeroCommand", "===============ZEROING SLIDES===============");
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > ArmConstants.ZEROING_TIMEOUT || hasStopped();
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

        Commands.waitMillis(30)
                .andThen(Commands.runOnce(() -> arm.slideZeroing = false))
                .schedule();

        Log.i("SlideZeroCommand", "===============SLIDE ZEROED===============");
    }

    private boolean hasStopped() {
        return timer.milliseconds() > 20 && Math.abs(arm.getSlideVelocity()) < ArmConstants.ZEROING_VELOCITY_ERROR;
    }
}
