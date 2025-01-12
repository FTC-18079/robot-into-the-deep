package org.firstinspires.ftc.teamcode.arm.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.ArmConstants;
import org.firstinspires.ftc.teamcode.util.commands.Commands;

@Config
public class PivotZeroCommand extends CommandBase {
    private final Arm arm;
    private final ElapsedTime timer;

    public static double COMMAND_TIMEOUT = 750;
    public static double PIVOT_POWER = -0.15;

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
        return timer.milliseconds() > COMMAND_TIMEOUT;
    }

    @Override
    public void execute() {
        arm.setPivotPower(PIVOT_POWER);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setPivotPower(0);
        arm.resetPivotEncoder();

        Commands.waitMillis(30)
                .andThen(Commands.runOnce(() -> arm.setPivotPos(arm.getPivotPos())))
                .andThen(Commands.runOnce(() -> arm.pivotZeroing = false))
                .schedule();
    }
}
