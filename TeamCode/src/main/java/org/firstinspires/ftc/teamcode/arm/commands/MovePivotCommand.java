package org.firstinspires.ftc.teamcode.arm.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.ArmConstants;

import java.util.function.DoubleSupplier;

public class MovePivotCommand extends CommandBase {
    private final Arm arm;
    private final double targetPos;
    private final ElapsedTime timer;
    boolean exceededTime = false;

    public MovePivotCommand(DoubleSupplier targetPos) {
        this.arm = Arm.getInstance();
        this.targetPos = targetPos.getAsDouble();
        timer = new ElapsedTime();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        timer.reset();
        arm.setPivotPos(targetPos);
    }

    @Override
    public void execute() {
        exceededTime = timer.milliseconds() > ArmConstants.PIVOT_TIMEOUT;
    }

    @Override
    public boolean isFinished() {
        return arm.pivotAtSetPoint() || exceededTime;
    }

    @Override
    public void end(boolean interrupted) {
        if (exceededTime) arm.setPivotPos(arm.getPivotPos());
    }
}
