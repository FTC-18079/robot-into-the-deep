package org.firstinspires.ftc.teamcode.util.commands;

import static org.firstinspires.ftc.teamcode.util.commands.ErrorMessages.requireNonNullParam;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.PrintCommand;
import com.arcrobotics.ftclib.command.Subsystem;

import java.util.function.Supplier;

public class DeferredCommand extends CommandBase {
    private final Command m_nullCommand =
            new PrintCommand("[DeferredCommand] Supplied command was null!");

    private final Supplier<Command> m_supplier;
    private Command m_command = m_nullCommand;

    public DeferredCommand(Supplier<Command> supplier, Subsystem... requirements) {
        m_supplier = requireNonNullParam(supplier, "supplier", "DeferredCommand");
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        Command cmd = m_supplier.get();
        if (cmd != null) {
            m_command = cmd;
        }
        m_command.initialize();
    }

    @Override
    public void execute() {
        m_command.execute();
    }

    @Override
    public boolean isFinished() {
        return m_command.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        m_command.end(interrupted);
        m_command = m_nullCommand;
    }
}
