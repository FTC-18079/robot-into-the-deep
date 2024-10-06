package org.firstinspires.ftc.teamcode.elevator.commands;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.elevator.Elevator;
import org.firstinspires.ftc.teamcode.elevator.ElevatorConstants;
import org.firstinspires.ftc.teamcode.util.commands.Commands;

import java.util.function.Supplier;

public class ElevatorCommands {
    public static final Supplier<Command> SCORE_BASKET_COMMAND;
    public static final Supplier<Command> SCORE_HIGH_CHAMBER_COMMAND;
    public static final Supplier<Command> RETRACT_COMMAND;
    public static final Supplier<Command> EJECT_COMMAND;
    public static Command SCORE_COMMAND;
    public static Command RELEASE_COMMAND;

    static {
        Supplier<Elevator> elevator = Elevator::getInstance;

        SCORE_BASKET_COMMAND = () -> Commands.sequence(
                Commands.runOnce(elevator.get()::scoreBucket),
                Commands.waitMillis(350),
                Commands.runOnce(elevator.get()::openDoor)
        );

        SCORE_HIGH_CHAMBER_COMMAND = () -> Commands.sequence(
                Commands.runOnce(elevator.get()::scoreChamberHigh),
                Commands.waitUntil(elevator.get()::atSetPoint),
                Commands.runOnce(elevator.get()::openClaw),
                Commands.waitMillis(75),
                Commands.runOnce(elevator.get()::toRest)
        );

        RETRACT_COMMAND = () -> Commands.sequence(
                Commands.runOnce(elevator.get()::returnBucket),
                Commands.runOnce(elevator.get()::toRest)
        );

        EJECT_COMMAND = () -> Commands.sequence(
                Commands.runOnce(elevator.get()::openClaw),
                Commands.waitMillis(75)
        );
    }

    static {
        Supplier<Elevator> elevator = Elevator::getInstance;

        SCORE_COMMAND = Commands.deferredProxy(() -> {
            if (elevator.get().getTargetPos() == ElevatorConstants.LIFT_POS_LOW_BASKET || elevator.get().getTargetPos() == ElevatorConstants.LIFT_POS_HIGH_BASKET) {
                return Commands.defer(SCORE_BASKET_COMMAND, elevator.get());
            } else if (elevator.get().getTargetPos() == ElevatorConstants.LIFT_POS_HIGH_CHAMBER) {
                return Commands.defer(SCORE_HIGH_CHAMBER_COMMAND, elevator.get());
            } else {
                return Commands.defer(EJECT_COMMAND, elevator.get());
            }
        });

        RELEASE_COMMAND = Commands.deferredProxy(() -> {
            if (elevator.get().getScoreType() == Elevator.ScoreType.SAMPLE) {
                return Commands.defer(RETRACT_COMMAND, elevator.get());
            } else {
                return Commands.none();
            }
        });
    }
}
