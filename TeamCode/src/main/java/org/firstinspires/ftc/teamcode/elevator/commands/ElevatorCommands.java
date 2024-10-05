package org.firstinspires.ftc.teamcode.elevator.commands;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.elevator.Elevator;
import org.firstinspires.ftc.teamcode.elevator.ElevatorConstants;
import org.firstinspires.ftc.teamcode.util.commands.Commands;

import java.util.function.Supplier;

public class ElevatorCommands {
    public static final Supplier<Command> SCORE_BASKET_COMMAND;
    public static final Supplier<Command> SCORE_HIGH_CHAMBER_COMMAND;
    public static final Supplier<Command> EJECT_COMMAND;
    public static Command SCORE_COMMAND;

    static {
        Elevator elevator = Elevator.getInstance();

        SCORE_BASKET_COMMAND = () -> Commands.sequence(
                Commands.runOnce(elevator::scoreBucket),
                Commands.waitMillis(350),
                Commands.runOnce(elevator::openDoor),
                Commands.waitMillis(150),
                Commands.runOnce(elevator::returnBucket),
                Commands.runOnce(elevator::closeDoor),
                Commands.runOnce(elevator::toRest)
        );

        SCORE_HIGH_CHAMBER_COMMAND = () -> Commands.sequence(
                Commands.runOnce(elevator::scoreChamberHigh),
                Commands.waitMillis(10),
                Commands.waitUntil(elevator::atSetPoint),
                Commands.runOnce(elevator::openClaw),
                Commands.waitMillis(75),
                Commands.runOnce(elevator::toRest)
        );

        EJECT_COMMAND = () -> Commands.sequence(
                Commands.runOnce(elevator::openClaw),
                Commands.waitMillis(75)
        );
    }

    static {
        Elevator elevator = Elevator.getInstance();

        SCORE_COMMAND = Commands.deferredProxy(() -> {
            if (elevator.getTargetPos() == ElevatorConstants.LIFT_POS_LOW_BASKET || elevator.getTargetPos() == ElevatorConstants.LIFT_POS_HIGH_BASKET) {
                return Commands.defer(SCORE_BASKET_COMMAND, elevator);
            } else if (elevator.getTargetPos() == ElevatorConstants.LIFT_POS_HIGH_CHAMBER) {
                return Commands.defer(SCORE_HIGH_CHAMBER_COMMAND, elevator);
            } else {
                return Commands.defer(EJECT_COMMAND, elevator);
            }
        });
    }
}
