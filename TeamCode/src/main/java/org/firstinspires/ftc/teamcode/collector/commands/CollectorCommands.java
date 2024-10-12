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

    static {
        Supplier<Collector> collector = Collector::getInstance;
        Supplier<Elevator> elevator = Elevator::getInstance;

        TO_STOW = () -> Commands.sequence(
                Commands.runOnce(elevator.get()::restBucket),
                Commands.waitMillis(75),
                Commands.runOnce(() -> collector.get().setPivot(CollectorConstants.PIVOT_PASSTHROUGH_POS)),
                Commands.runOnce(collector.get()::toRest)
        );

        TO_COLLECTING = () -> Commands.sequence(
                // TODO: make a method for going to limelight pose
                Commands.runOnce(() -> collector.get().setTargetPose(1000)),
                Commands.waitUntil(collector.get()::atSetPoint),
                Commands.runOnce(collector.get()::deploySeek),
                Commands.waitMillis(50),
                Commands.runOnce(elevator.get()::passthroughBucket),
                Commands.runOnce(elevator.get()::closeDoor)
        );

        COLLECT = () -> Commands.sequence(
                Commands.runOnce(collector.get()::deployCollect),
                Commands.waitMillis(50),
                Commands.runOnce(collector.get()::grab),
                Commands.waitMillis(25)
        );

        TO_PASSTHROUGH = () -> Commands.sequence(
                Commands.runOnce(() -> collector.get().setPivot(CollectorConstants.PIVOT_PASSTHROUGH_POS)),
                Commands.runOnce(collector.get()::deployStow),
                Commands.waitMillis(50),
                Commands.runOnce(collector.get()::toPassthrough),
                Commands.waitUntil(collector.get()::atSetPoint),
                Commands.runOnce(elevator.get()::openDoor),
                Commands.waitMillis(75),
                Commands.runOnce(elevator.get()::closeDoor),
                Commands.waitMillis(75)
        );
    }

    static {
        Supplier<Collector> collector = Collector::getInstance;
        Supplier<Elevator> elevator = Elevator::getInstance;
    }
}
