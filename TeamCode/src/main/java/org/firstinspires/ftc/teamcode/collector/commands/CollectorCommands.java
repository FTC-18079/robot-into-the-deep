package org.firstinspires.ftc.teamcode.collector.commands;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.collector.Collector;
import org.firstinspires.ftc.teamcode.util.commands.Commands;

import java.util.function.Supplier;

public class CollectorCommands {
    public static final Supplier<Command> PASSTHROUGH_COMMAND;

    static {
        Supplier<Collector> collector = Collector::getInstance;

        PASSTHROUGH_COMMAND = () -> Commands.sequence(

        );
    }
}
