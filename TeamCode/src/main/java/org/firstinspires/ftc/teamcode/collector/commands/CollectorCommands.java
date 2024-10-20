package org.firstinspires.ftc.teamcode.collector.commands;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.collector.Collector;
import org.firstinspires.ftc.teamcode.collector.CollectorConstants;
import org.firstinspires.ftc.teamcode.elevator.Elevator;
import org.firstinspires.ftc.teamcode.util.commands.Commands;

import java.util.function.Supplier;

public class CollectorCommands {
    public static final Supplier<Command> TO_STOW;
    public static final Supplier<Command> TO_COLLECTING;
    public static final Supplier<Command> COLLECT;
    public static final Supplier<Command> TO_PASSTHROUGH;
    public static Command COLLECT_SEQUENCE;

    static {
        Supplier<Collector> collector = Collector::getInstance;
        Supplier<Elevator> elevator = Elevator::getInstance;

        TO_STOW = () -> Commands.sequence(
                Commands.runOnce(() -> collector.get().setState(Collector.CollectorState.STOW)),
                Commands.runOnce(elevator.get()::restBucket),
                Commands.waitMillis(100),
                Commands.runOnce(collector.get()::toRest),
                Commands.waitUntil(collector.get()::atSetPoint),
                Commands.runOnce(collector.get()::deployStow)
        );

        TO_COLLECTING = () -> Commands.sequence(
                Commands.runOnce(() -> collector.get().setTargetPose(CollectorConstants.SLIDE_COLLECTING_POS)),
                Commands.waitUntil(collector.get()::atSetPoint),
                Commands.runOnce(collector.get()::deploySeek),
                Commands.runOnce(collector.get()::release),
                Commands.waitMillis(50),
                Commands.runOnce(elevator.get()::passthroughBucket),
                Commands.runOnce(elevator.get()::openDoor),
                Commands.runOnce(() -> collector.get().setState(Collector.CollectorState.COLLECTING))
        );

        COLLECT = () -> Commands.sequence(
                Commands.runOnce(() -> collector.get().setState(Collector.CollectorState.COLLECT)),
                Commands.runOnce(() -> collector.get().setTargetPose(CollectorConstants.SLIDE_COLLECT_POS)), // replace for toCollect
                Commands.waitUntil(collector.get()::atSetPoint),
                Commands.runOnce(collector.get()::deployCollect),
                Commands.waitMillis(400),
                Commands.runOnce(collector.get()::grab),
                Commands.waitMillis(275)
        );

        TO_PASSTHROUGH = () -> Commands.sequence(
                Commands.runOnce(() -> collector.get().setState(Collector.CollectorState.PASSTHROUGH)),
                Commands.runOnce(collector.get()::deployStow),
                Commands.waitMillis(50),
                Commands.runOnce(collector.get()::toPassthrough),
                Commands.waitUntil(collector.get()::atSetPoint),
                Commands.runOnce(collector.get()::release),
                Commands.waitMillis(150),
                Commands.runOnce(elevator.get()::closeDoor),
                Commands.waitMillis(100),
                Commands.runOnce(elevator.get()::restBucket),
                Commands.waitMillis(250)
        );
    }

    static {
        Supplier<Collector> collector = Collector::getInstance;
        Supplier<Elevator> elevator = Elevator::getInstance;

        COLLECT_SEQUENCE = Commands.deferredProxy(() -> Commands.sequence(
                Commands.defer(COLLECT, collector.get()),
                Commands.defer(TO_PASSTHROUGH, collector.get(), elevator.get()),
                Commands.defer(TO_STOW, collector.get(), elevator.get())
        ));
    }
}
