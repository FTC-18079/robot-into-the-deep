package org.firstinspires.ftc.teamcode.arm.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.ArmConstants;

import java.util.function.DoubleSupplier;

public class MoveSlideCommand extends CommandBase {
    private final Arm arm;
    private final double targetPos;
    private final ElapsedTime timer;
    boolean exceededTime = false;

    public MoveSlideCommand(DoubleSupplier targetPos) {
        this.arm = Arm.getInstance();
        this.targetPos = targetPos.getAsDouble();
        timer = new ElapsedTime();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        timer.reset();
        arm.setSlidePos(targetPos);
    }

    @Override
    public void execute() {
        exceededTime = timer.milliseconds() > ArmConstants.SLIDE_TIMEOUT;
    }

    @Override
    public boolean isFinished() {
        return arm.slideAtSetPoint() || exceededTime;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            Log.i("MoveSlideCommand", "===============COMMAND INTERRUPTED===============");
        } else if (exceededTime) {
            arm.setSlidePos(arm.getSlidePos());
            Log.i("MoveSlideCommand", "===============COMMAND TIMED OUT AT " + timer.milliseconds() + " MILLISECONDS===============");
        } else {
            Log.i("MoveSlideCommand", "===============COMMAND ENDED SAFELY===============");
        }
    }
}
