package org.firstinspires.ftc.teamcode.arm.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.ArmConstants;

import java.util.function.DoubleSupplier;

public class MovePivotCommand extends CommandBase {
    private final Arm arm;
    private final double targetPos;
    private final ElapsedTime timer;

    public MovePivotCommand(DoubleSupplier targetPos) {
        this.arm = Arm.getInstance();
        this.targetPos = targetPos.getAsDouble();
        timer = new ElapsedTime();
    }

    @Override
    public void initialize() {
        timer.reset();
        arm.setPivotPos(targetPos);
    }

    @Override
    public boolean isFinished() {
        return arm.pivotAtSetPoint() || timer.milliseconds() > ArmConstants.PIVOT_TIMEOUT;
    }

    @Override
    public void end(boolean interrupted) {
        if (targetPos == ArmConstants.PIVOT_REST_POSITION && arm.pivotAtSetPoint()) {
            arm.resetPivotEncoder();
        }
        arm.setPivotPos(arm.getPivotPos());
    }
}
