package org.firstinspires.ftc.teamcode.arm.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.ArmConstants;
import org.firstinspires.ftc.teamcode.util.commands.Commands;

public class PivotZeroCommand extends CommandBase {
    private final Arm arm;
    private final ElapsedTime timer;

    public PivotZeroCommand() {
        arm = Arm.getInstance();
        timer = new ElapsedTime();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        timer.reset();
        arm.pivotZeroing = true;
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > ArmConstants.ZEROING_TIMEOUT;
    }

    @Override
    public void execute() {
        arm.setPivotPower(-0.35);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setPivotPower(0);
        arm.resetPivotEncoder();
        arm.setPivotPos(0);

        Commands.waitMillis(30)
                .andThen(Commands.runOnce(() -> arm.pivotZeroing = false))
                .schedule();
    }
}
