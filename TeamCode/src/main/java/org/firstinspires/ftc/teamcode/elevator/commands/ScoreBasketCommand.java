package org.firstinspires.ftc.teamcode.elevator.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.elevator.Elevator;
import org.firstinspires.ftc.teamcode.elevator.ElevatorConstants;

public class ScoreBasketCommand extends SequentialCommandGroup {
    Elevator elevator;

    public ScoreBasketCommand() {
        this.elevator = Elevator.getInstance();

        addCommands(
                new ConditionalCommand(
                        // Do nothing of not scoring basket
                        new InstantCommand(),
                        // Otherwise score
                        new InstantCommand(elevator::scoreBucket)
                                .andThen(new WaitCommand(350))
                                .andThen(new InstantCommand(elevator::openDoor))
                                .andThen(new WaitCommand(150))
                                .andThen(new InstantCommand(elevator::closeDoor))
                                .andThen(new InstantCommand(elevator::returnBucket)),
                        // Condition to check whether or not to run the command
                        () -> (elevator.getTargetPos() != ElevatorConstants.LIFT_POS_HIGH_BASKET) || (elevator.getTargetPos() != ElevatorConstants.LIFT_POS_LOW_BASKET)
                )
        );

        addRequirements(elevator);
    }
}
