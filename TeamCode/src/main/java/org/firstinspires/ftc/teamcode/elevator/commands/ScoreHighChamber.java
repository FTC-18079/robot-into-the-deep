package org.firstinspires.ftc.teamcode.elevator.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.elevator.Elevator;
import org.firstinspires.ftc.teamcode.elevator.ElevatorConstants;

public class ScoreHighChamber extends SequentialCommandGroup {
    Elevator elevator;

    public ScoreHighChamber() {
        this.elevator = Elevator.getInstance();

        addCommands(
                new ConditionalCommand(
                        // Run nothing if not scoring low chamber
                        new InstantCommand(),
                        // Otherwise score
                        new InstantCommand(elevator::scoreChamberHigh)
                                .andThen(new WaitUntilCommand(elevator::atSetPoint))
                                .andThen(new InstantCommand(elevator::openClaw))
                                .andThen(new WaitCommand(100)),
                        // Condition to check whether or not to run the command
                        () -> elevator.getTargetPos() != ElevatorConstants.LIFT_POS_HIGH_CHAMBER
                )
        );

        addRequirements(elevator);
    }
}
