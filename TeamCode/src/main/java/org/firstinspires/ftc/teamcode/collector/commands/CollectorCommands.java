package org.firstinspires.ftc.teamcode.collector.commands;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.collector.Collector;
import org.firstinspires.ftc.teamcode.elevator.Elevator;
import org.firstinspires.ftc.teamcode.util.commands.Commands;

import java.util.function.Supplier;

public class CollectorCommands {
    public static final Supplier<Command> PASSTHROUGH_COMMAND;

    static {
        Supplier<Collector> collector = Collector::getInstance;
        Supplier<Elevator> elevator = Elevator::getInstance;

        PASSTHROUGH_COMMAND = () -> Commands.sequence(
                Commands.runOnce(collector.get()::release),
                Commands.runOnce(elevator.get()::openDoor),
                Commands.waitMillis(200),
                Commands.runOnce(elevator.get()::closeDoor)
        );
    }

    static {
        Supplier<Collector> collector = Collector::getInstance;
        Supplier<Elevator> elevator = Elevator::getInstance;
    }
}
