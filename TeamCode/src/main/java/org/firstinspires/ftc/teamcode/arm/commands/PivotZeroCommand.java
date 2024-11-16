package org.firstinspires.ftc.teamcode.arm.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.arm.Arm;

public class PivotZeroCommand extends CommandBase {
    private final Arm arm;
    private final ElapsedTime timer;

    public PivotZeroCommand() {
        arm = Arm.getInstance();
        timer = new ElapsedTime();
    }

    @Override
    public void initialize() {
        timer.reset();
        arm.pivotZeroing = true;
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > 300;
    }

    @Override
    public void execute() {
        arm.setPivotPower(-0.5);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setPivotPower(0);
        arm.resetPivotEncoder();
        arm.setPivotPos(0);
        arm.pivotZeroing = false;
    }
}
