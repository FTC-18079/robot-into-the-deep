package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.PrintCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class CommandFactory {
    public static Command none() {
        return new InstantCommand();
    }

    public static Command idle(Subsystem... requirements) {
        return run(() -> {}, requirements);
    }

    public static Command runOnce(Runnable action, Subsystem... requirements) {
        return new InstantCommand(action, requirements);
    }

    public static Command run(Runnable action, Subsystem... requirements) {
        return new RunCommand(action, requirements);
    }

    public static Command startEnd(Runnable start, Runnable end, Subsystem... requirements) {
        return new StartEndCommand(start, end, requirements);
    }

    public static Command runEnd(Runnable run, Runnable end, Subsystem... requirements) {
//        requireNonNullParam(end, "end", "Command.runEnd");
        return new FunctionalCommand(() -> {}, run, inerrupted -> end.run(), () -> false, requirements);
    }

    public static Command print(String message) {
        return new PrintCommand(message);
    }

    public static Command waitSeconds(double seconds) {
        return new WaitCommand(seconds);
    }

    public static Command waitUntil(BooleanSupplier condition) {
        return new WaitUntilCommand(condition);
    }

    public static Command either(Command onTrue, Command onFalse, BooleanSupplier selector) {
        return new ConditionalCommand(onTrue, onFalse, selector);
    }

//    public static <K> Command select(Map<K, Command> commands, Supplier<? extends K> selector) {
//        return new SelectCommand<>(commands, selector);
//    }

//    public static Command defer(Supplier<Command> supplier, Set<Subsystem> requirements) {
//        return new DeferredCommand(supplier, requirements);
//    }

//    public static Command deferredProxy(Supplier<Command> supplier) {
//        return new ProxyCommand(supplier);
//    }

    public static Command sequence(Command... commands) {
        return new SequentialCommandGroup(commands);
    }

//    public static Command repeatingSequence(Command... commands) {
//        return sequence(commands).repeatedly();
//    }

    public static Command parallel(Command... commands) {
        return new ParallelCommandGroup(commands);
    }

    public static Command race(Command... commands) {
        return new ParallelRaceGroup(commands);
    }

    public static Command deadline(Command deadline, Command... otherCommands) {
        return new ParallelDeadlineGroup(deadline, otherCommands);
    }

    private CommandFactory() {
        throw new UnsupportedOperationException("This is a utility class");
    }
}
