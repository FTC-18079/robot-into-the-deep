package org.firstinspires.ftc.teamcode.arm.commands;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.ArmConstants;
import org.firstinspires.ftc.teamcode.claw.Claw;
import org.firstinspires.ftc.teamcode.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.util.commands.Commands;

import java.util.function.Supplier;

public class ArmCommands {
    // TO STOW
    public static final Supplier<Command> BASKET_TO_STOW;
    public static final Supplier<Command> CHAMBER_TO_STOW;
    public static final Supplier<Command> SAMPLE_COLLECT_TO_STOW;
    public static final Supplier<Command> SPECIMEN_COLLECT_TO_STOW;

    // TO BASKET
    public static final Supplier<Command> STOW_TO_BASKET;
    public static final Supplier<Command> CHAMBER_TO_BASKET;

    // TO CHAMBER
    public static final Supplier<Command> SPECIMEN_COLLECT_TO_CHAMBER;
    public static final Supplier<Command> BASKET_TO_CHAMBER;

    // TO SAMPLE COLLECT
    public static final Supplier<Command> STOW_TO_SAMPLE_COLLECT;
    public static final Supplier<Command> COLLECT;

    // TO SPECIMEN COLLECT
    public static final Supplier<Command> STOW_TO_SPECIMEN_COLLECT;

    // ACTIONS
    public static final Supplier<Command> GRAB;
    public static final Supplier<Command> RELEASE;
    public static final Supplier<Command> SCORE_SPECIMEN;

    // MOVEMENT COMMANDS
    public static Command TO_STOW;
    public static Command TO_BASKET;
    public static Command TO_CHAMBER;
    public static Command TO_COLLECT;
    public static Command ARM_ACTION;

    static {
        Supplier<Arm> arm = Arm::getInstance;
        Supplier<Claw> claw = Claw::getInstance;

        BASKET_TO_STOW = () -> Commands.sequence(
                Commands.runOnce(() -> arm.get().setState(Arm.ArmState.STOW)),
                Commands.runOnce(() -> claw.get().setJointOne(0.3)),
                Commands.waitMillis(100),
                Commands.runOnce(() -> arm.get().setSlidePos(ArmConstants.SLIDE_REST_POSITION)),
                Commands.waitUntil(arm.get()::slideAtSetPoint),
                Commands.runOnce(() -> claw.get().setState(ClawConstants.REST_STATE)),
                Commands.runOnce(() -> arm.get().setPivotPos(ArmConstants.PIVOT_REST_POSITION)),
                Commands.waitUntil(arm.get()::pivotAtSetPoint)
        );
        CHAMBER_TO_STOW = () -> Commands.sequence(
                Commands.runOnce(() -> arm.get().setState(Arm.ArmState.STOW)),
                Commands.runOnce(() -> claw.get().setState(ClawConstants.REST_STATE)),
                Commands.runOnce(() -> arm.get().setSlidePos(ArmConstants.SLIDE_ERROR_TOLERANCE)),
                Commands.runOnce(() -> arm.get().setPivotPos(ArmConstants.PIVOT_REST_POSITION)),
                Commands.waitUntil(() -> arm.get().slideAtSetPoint() && arm.get().pivotAtSetPoint())
        );
        SAMPLE_COLLECT_TO_STOW = () -> Commands.sequence(
                Commands.runOnce(() -> arm.get().setState(Arm.ArmState.STOW)),
                Commands.runOnce(() -> claw.get().setState(ClawConstants.REST_STATE)),
                Commands.runOnce(() -> arm.get().setSlidePos(ArmConstants.SLIDE_REST_POSITION)),
                Commands.waitUntil(arm.get()::slideAtSetPoint)
        );
        SPECIMEN_COLLECT_TO_STOW = () -> Commands.sequence(
                Commands.runOnce(() -> arm.get().setState(Arm.ArmState.STOW)),
                Commands.runOnce(() -> arm.get().setSlidePos(ArmConstants.SLIDE_REST_POSITION)),
                Commands.runOnce(() -> claw.get().setState(ClawConstants.REST_STATE)),
                Commands.waitUntil(arm.get()::slideAtSetPoint)
        );

        STOW_TO_BASKET = () -> Commands.sequence(
                Commands.runOnce(() -> arm.get().setState(Arm.ArmState.SCORING_SAMPLE)),
                Commands.runOnce(() -> arm.get().setPivotPos(ArmConstants.PIVOT_SCORE_POSITION)),
                Commands.runOnce(() -> claw.get().setState(ClawConstants.SAMPLE_SCORING_STATE)),
                Commands.waitUntil(arm.get()::pivotAtSetPoint),
                Commands.runOnce(() -> arm.get().setSlidePos(ArmConstants.SLIDE_BASKET_POSITION)),
                Commands.waitUntil(arm.get()::slideAtSetPoint)
        );
        CHAMBER_TO_BASKET = () -> Commands.sequence(
                Commands.runOnce(() -> arm.get().setState(Arm.ArmState.SCORING_SAMPLE)),
                Commands.runOnce(() -> arm.get().setSlidePos(ArmConstants.SLIDE_BASKET_POSITION)),
                Commands.waitUntil(arm.get()::slideAtSetPoint)
        );

        SPECIMEN_COLLECT_TO_CHAMBER = () -> Commands.sequence(
                Commands.runOnce(() -> claw.get().setJointOne(0.5)),
                Commands.waitMillis(100),
                Commands.runOnce(() -> arm.get().setState(Arm.ArmState.SCORING_SPECIMEN)),
                Commands.runOnce(() -> arm.get().setSlidePos(ArmConstants.SLIDE_REST_POSITION)),
                Commands.waitUntil(arm.get()::slideAtSetPoint),
                Commands.runOnce(() -> claw.get().setState(ClawConstants.SPECIMEN_SCORING_STATE)),
                Commands.runOnce(() -> arm.get().setPivotPos(ArmConstants.PIVOT_SCORE_POSITION)),
                Commands.waitUntil(arm.get()::pivotAtSetPoint),
                Commands.runOnce(() -> arm.get().setSlidePos(ArmConstants.SLIDE_CHAMBER_POSITION)),
                Commands.waitUntil(arm.get()::slideAtSetPoint)
        );

        BASKET_TO_CHAMBER = () -> Commands.sequence(
                Commands.runOnce(() -> arm.get().setState(Arm.ArmState.SCORING_SPECIMEN)),
                Commands.runOnce(() -> arm.get().setSlidePos(ArmConstants.SLIDE_CHAMBER_POSITION)),
                Commands.waitUntil(arm.get()::slideAtSetPoint)
        );

        STOW_TO_SAMPLE_COLLECT = () -> Commands.sequence(
                Commands.runOnce(() -> arm.get().setSlidePos(ArmConstants.SLIDE_SAMPLE_COLLECT_POSITION)),
                Commands.waitUntil(arm.get()::slideAtSetPoint),
                Commands.runOnce(() -> arm.get().setState(Arm.ArmState.COLLECTING_SAMPLE)),
                Commands.runOnce(() -> claw.get().setState(ClawConstants.SAMPLE_COLLECTING_STATE))
        );

        STOW_TO_SPECIMEN_COLLECT = () -> Commands.sequence(
                Commands.runOnce(() -> claw.get().setState(ClawConstants.SPECIMEN_COLLECT_STATE)),
                Commands.runOnce(() -> arm.get().setSlidePos(ArmConstants.SLIDE_SPECIMEN_COLLECT_POSITION)),
                Commands.waitUntil(arm.get()::slideAtSetPoint),
                Commands.runOnce(() -> arm.get().setState(Arm.ArmState.COLLECTING_SPECIMEN))
        );

        GRAB = () -> Commands.sequence(
                Commands.runOnce(claw.get()::closeClaw),
                Commands.waitMillis(375)
        );

        RELEASE = () -> Commands.sequence(
                Commands.runOnce(claw.get()::openClaw),
                Commands.waitMillis(500)
        );

        SCORE_SPECIMEN = () -> Commands.sequence(
                Commands.runOnce(() -> claw.get().setState(ClawConstants.SPECIMEN_SCORE_STATE)),
                Commands.waitMillis(150),
                Commands.runOnce(claw.get()::openClaw),
                Commands.waitMillis(100)
        );

        COLLECT = () -> Commands.sequence(
                Commands.runOnce(() -> claw.get().setJointOne(ClawConstants.SAMPLE_COLLECT_JOINT_ONE_POS)),
                Commands.waitMillis(100),
                Commands.runOnce(() -> claw.get().setJointTwo(ClawConstants.SAMPLE_COLLECT_JOINT_TWO_POS)),
                Commands.waitMillis(250)
        );

    }

    static {
        Supplier<Arm> arm = Arm::getInstance;
        Supplier<Claw> claw = Claw::getInstance;

        TO_STOW = Commands.deferredProxy(() -> {
            switch (arm.get().getState()) {
                case COLLECTING_SAMPLE:
                    return Commands.defer(SAMPLE_COLLECT_TO_STOW, arm.get());
                case SCORING_SPECIMEN:
                    return Commands.defer(CHAMBER_TO_STOW, arm.get());
                case SCORING_SAMPLE:
                    return Commands.defer(BASKET_TO_STOW, arm.get());
                case COLLECTING_SPECIMEN:
                    return Commands.defer(SPECIMEN_COLLECT_TO_STOW, arm.get());
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
            if (arm.get().getState() == Arm.ArmState.STOW) {
                if (arm.get().getScoreType() == Arm.ScoreType.SPECIMEN) {
                    return Commands.defer(STOW_TO_SPECIMEN_COLLECT, arm.get());
                } else {
                    return Commands.defer(STOW_TO_SAMPLE_COLLECT, arm.get());
                }
            } else return Commands.none();
        });

        ARM_ACTION = Commands.deferredProxy(() -> {
            switch (arm.get().getState()) {
                case COLLECTING_SAMPLE:
                    return Commands.sequence(
                            Commands.defer(COLLECT, claw.get()),
                            Commands.defer(GRAB, claw.get()),
                            Commands.defer(SAMPLE_COLLECT_TO_STOW, arm.get())
                    );
                case COLLECTING_SPECIMEN:
                    return Commands.sequence(
                            Commands.defer(GRAB, claw.get()),
                            Commands.defer(SPECIMEN_COLLECT_TO_CHAMBER, arm.get())
                    );
                case SCORING_SAMPLE:
                    return Commands.sequence(
                            Commands.defer(RELEASE, claw.get()),
                            Commands.defer(BASKET_TO_STOW, arm.get())
                    );
                case SCORING_SPECIMEN:
                    return Commands.sequence(
                            Commands.defer(SCORE_SPECIMEN, arm.get(), claw.get()),
                            Commands.defer(CHAMBER_TO_STOW, arm.get())
                    );
                case STOW:
                    return Commands.either(
                            Commands.defer(STOW_TO_SAMPLE_COLLECT),
                            Commands.defer(STOW_TO_SPECIMEN_COLLECT),
                            () -> arm.get().getScoreType() == Arm.ScoreType.SAMPLE
                    );
                default:
                    return Commands.none();
            }
        });
    }
}
