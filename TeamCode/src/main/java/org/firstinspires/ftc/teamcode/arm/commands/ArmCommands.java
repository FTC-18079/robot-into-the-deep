package org.firstinspires.ftc.teamcode.arm.commands;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.ArmConstants;
import org.firstinspires.ftc.teamcode.util.commands.Commands;

import java.util.function.Supplier;

public class ArmCommands {
    // TO STOW
    public static final Supplier<Command> BASKET_TO_STOW;
    public static final Supplier<Command> CHAMBER_TO_STOW;
    public static final Supplier<Command> SAMPLE_COLLECT_TO_STOW;

    // TO BASKET
    public static final Supplier<Command> STOW_TO_BASKET;
    public static final Supplier<Command> CHAMBER_TO_BASKET;

    // TO CHAMBER
    public static final Supplier<Command> SPECIMEN_COLLECT_TO_CHAMBER;
    public static final Supplier<Command> BASKET_TO_CHAMBER;

    // TO SAMPLE COLLECT
    public static final Supplier<Command> STOW_TO_SAMPLE_COLLECT;

    // TO SPECIMEN COLLECT
    public static final Supplier<Command> STOW_TO_SPECIMEN_COLLECT;

    // MOVEMENT COMMANDS
    public static Command TO_STOW;
    public static Command TO_BASKET;
    public static Command TO_CHAMBER;
    public static Command TO_COLLECT;

    static {
        Supplier<Arm> arm = Arm::getInstance;

        BASKET_TO_STOW = () -> Commands.sequence(
                Commands.runOnce(() -> arm.get().setSlidePos(0)),
                Commands.waitUntil(() -> arm.get().getSlidePos() < 1000.0),
                Commands.runOnce(() -> arm.get().setPivotPos(0)),
                Commands.waitUntil(() -> arm.get().slideAtSetPoint() && arm.get().pivotAtSetPoint())
        );
        CHAMBER_TO_STOW = () -> Commands.sequence(
                Commands.runOnce(() -> arm.get().setSlidePos(0)),
                Commands.runOnce(() -> arm.get().setPivotPos(0)),
                Commands.waitUntil(() -> arm.get().slideAtSetPoint() && arm.get().pivotAtSetPoint())
        );
        SAMPLE_COLLECT_TO_STOW = () -> Commands.sequence();

        STOW_TO_BASKET = () -> Commands.sequence(
                Commands.runOnce(() -> arm.get().setPivotPos(ArmConstants.PIVOT_SCORE_POSITION)),
                Commands.waitUntil(() -> arm.get().pivotAtSetPoint()),
                Commands.runOnce(() -> arm.get().setSlidePos(ArmConstants.SLIDE_BASKET_POSITION)),
                Commands.waitUntil(() -> arm.get().slideAtSetPoint())
        );
        CHAMBER_TO_BASKET = () -> Commands.sequence(
                Commands.runOnce(() -> arm.get().setSlidePos(ArmConstants.SLIDE_BASKET_POSITION)),
                Commands.waitUntil(() -> arm.get().slideAtSetPoint())
        );

        SPECIMEN_COLLECT_TO_CHAMBER = () -> Commands.sequence();
        BASKET_TO_CHAMBER = () -> Commands.sequence(
                Commands.runOnce(() -> arm.get().setSlidePos(ArmConstants.SLIDE_CHAMBER_POSITION)),
                Commands.waitUntil(() -> arm.get().slideAtSetPoint())
        );

        STOW_TO_SAMPLE_COLLECT = () -> Commands.sequence();

        STOW_TO_SPECIMEN_COLLECT = () -> Commands.sequence();
    }

    static {
        Supplier<Arm> arm = Arm::getInstance;

        TO_STOW = Commands.deferredProxy(() -> {
            switch (arm.get().getState()) {
                case COLLECTING_SAMPLE:
                    return Commands.defer(SAMPLE_COLLECT_TO_STOW, arm.get());
                case SCORING_SPECIMEN:
                    return Commands.defer(CHAMBER_TO_STOW, arm.get());
                case SCORING_SAMPLE:
                    return Commands.defer(BASKET_TO_STOW, arm.get());
                default:
                    return Commands.none();
            }
        });

        TO_BASKET = Commands.deferredProxy(() -> {
            if (arm.get().getScoreType() == Arm.ScoreType.SPECIMEN) return Commands.none();
            switch (arm.get().getState()) {
                case STOW:
                    return Commands.defer(STOW_TO_BASKET, arm.get());
                case SCORING_SPECIMEN:
                    return Commands.defer(CHAMBER_TO_BASKET, arm.get());
                default:
                    return Commands.none();
            }
        });

        TO_CHAMBER = Commands.deferredProxy(() -> {
            if (arm.get().getScoreType() == Arm.ScoreType.SAMPLE) return Commands.none();
            switch (arm.get().getState()) {
                case COLLECTING_SPECIMEN:
                    return Commands.defer(SPECIMEN_COLLECT_TO_CHAMBER, arm.get());
                case SCORING_SAMPLE:
                    return Commands.defer(BASKET_TO_CHAMBER, arm.get());
                default:
                    return Commands.none();
            }
        });

        TO_COLLECT = Commands.deferredProxy(() -> {
            if (arm.get().getState() == Arm.State.STOW) {
                if (arm.get().getScoreType() == Arm.ScoreType.SPECIMEN) {
                    return Commands.defer(STOW_TO_SPECIMEN_COLLECT, arm.get());
                } else {
                    return Commands.defer(STOW_TO_SAMPLE_COLLECT, arm.get());
                }
            } else return Commands.none();
        });
    }
}
