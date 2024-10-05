package org.firstinspires.ftc.teamcode.util.commands;

import static org.firstinspires.ftc.teamcode.util.commands.ErrorMessages.requireNonNullParam;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.Supplier;

public class ProxyCommand extends CommandBase {
    private final Supplier<Command> m_supplier;
    private Command m_command;

    public ProxyCommand(Supplier<Command> supplier) {
        m_supplier = requireNonNullParam(supplier, "supplier", "ProxyCommand");
    }

    @Override
    public void initialize() {
        m_command = m_supplier.get();
        m_command.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_command.cancel();
        }
        m_command = null;
    }

    @Override
    public boolean isFinished() {
        // because we're between `initialize` and `end`, `m_command` is necessarily not null
        // but if called otherwise and m_command is null,
        // it's UB, so we can do whatever we want -- like return true.
        return m_command == null || !m_command.isScheduled();
    }
}
