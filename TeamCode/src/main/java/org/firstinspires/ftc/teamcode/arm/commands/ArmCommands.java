package org.firstinspires.ftc.teamcode.arm.commands;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.ArmConstants;
import org.firstinspires.ftc.teamcode.claw.Claw;
import org.firstinspires.ftc.teamcode.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.util.commands.Commands;
import org.firstinspires.ftc.teamcode.vision.LLVision;

import java.util.function.Supplier;

public class ArmCommands {
    // TO ASCENT
    public static final Supplier<Command> BASKET_TO_AUTO_ASCENT;

    // TO STOW
    public static final Supplier<Command> BASKET_TO_STOW;
    public static final Supplier<Command> CHAMBER_TO_STOW;
    public static final Supplier<Command> SAMPLE_COLLECT_TO_STOW;
    public static final Supplier<Command> SPECIMEN_COLLECT_TO_STOW;

    // TO BASKET
    public static final Supplier<Command> STOW_TO_BASKET;
    public static final Supplier<Command> CHAMBER_TO_BASKET;

    // TO CHAMBER
    public static final Supplier<Command> STOW_TO_CHAMBER;
    public static final Supplier<Command> SPECIMEN_COLLECT_TO_CHAMBER;
    public static final Supplier<Command> BASKET_TO_CHAMBER;

    // TO SAMPLE COLLECT
    public static final Supplier<Command> STOW_TO_SAMPLE_COLLECT;
    public static final Supplier<Command> COLLECT_SAMPLE;
    public static final Supplier<Command> MISSED_SEQUENCE;

    // TO SPECIMEN COLLECT
    public static final Supplier<Command> STOW_TO_SPECIMEN_COLLECT;
    public static final Supplier<Command> CHAMBER_TO_SPECIMEN_COLLECT;

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

    public static Command CLOSE_COLLECT;

    static {
        Arm arm = Arm.getInstance();
        Claw claw = Claw.getInstance();
        LLVision limelight = LLVision.getInstance();

        BASKET_TO_STOW = () -> Commands.sequence(
                Commands.runOnce(() -> arm.setState(Arm.ArmState.STOW)),
                Commands.runOnce(() -> claw.setJointOne(0.3)),
                Commands.waitMillis(100),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_REST_POSITION),
                Commands.runOnce(() -> claw.setState(ClawConstants.REST_STATE)),
                new MovePivotCommand(() -> ArmConstants.PIVOT_REST_POSITION)
        );
        CHAMBER_TO_STOW = () -> Commands.sequence(
                Commands.runOnce(() -> arm.setState(Arm.ArmState.STOW)),
                Commands.runOnce(() -> claw.setState(ClawConstants.REST_STATE)),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_REST_POSITION),
                new MovePivotCommand(() -> ArmConstants.PIVOT_REST_POSITION)
        );
        SAMPLE_COLLECT_TO_STOW = () -> Commands.sequence(
                Commands.runOnce(() -> arm.setState(Arm.ArmState.STOW)),
//                Commands.runOnce(() -> claw.get().setState(ClawConstants.SPECIMEN_COLLECT_STATE)),
//                Commands.runOnce(claw.get()::closeClaw),
//                Commands.waitMillis(175),
                Commands.parallel(
                        new MoveSlideCommand(() -> ArmConstants.SLIDE_REST_POSITION),
                        Commands.waitMillis(75)
                                .andThen(Commands.runOnce(() -> claw.setState(ClawConstants.SPECIMEN_COLLECT_STATE)))
                                .andThen(Commands.runOnce(claw::closeClaw))
                ),
                Commands.runOnce(() -> claw.setState(ClawConstants.REST_STATE)),
                Commands.waitMillis(150),
                new SlideZeroCommand()
        );
        SPECIMEN_COLLECT_TO_STOW = () -> Commands.sequence(
                Commands.runOnce(() -> arm.setState(Arm.ArmState.STOW)),
                Commands.runOnce(() -> claw.setState(ClawConstants.REST_STATE)),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_REST_POSITION),
                new SlideZeroCommand()
        );

        STOW_TO_BASKET = () -> Commands.sequence(
                Commands.runOnce(() -> arm.setState(Arm.ArmState.SCORING_SAMPLE)),
                Commands.runOnce(() -> claw.setJointOne(0.6)),
                Commands.waitMillis(200),
                new MovePivotCommand(() -> ArmConstants.PIVOT_SCORE_POSITION),
                Commands.runOnce(() -> claw.setWrist(ClawConstants.SAMPLE_SCORING_STATE.wristPos)),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_BASKET_POSITION),
                Commands.runOnce(() -> claw.setState(ClawConstants.SAMPLE_SCORING_STATE)),
                Commands.waitMillis(200)
        );
        CHAMBER_TO_BASKET = () -> Commands.sequence(
                Commands.runOnce(() -> arm.setState(Arm.ArmState.SCORING_SAMPLE)),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_BASKET_POSITION)
        );

        SPECIMEN_COLLECT_TO_CHAMBER = () -> Commands.sequence(
                Commands.runOnce(() -> claw.setJointOne(0.65)),
                Commands.waitMillis(100),
                Commands.runOnce(() -> arm.setState(Arm.ArmState.SCORING_SPECIMEN)),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_REST_POSITION),
                Commands.runOnce(() -> claw.setState(ClawConstants.SPECIMEN_SCORING_STATE)),
                new MovePivotCommand(() -> ArmConstants.PIVOT_SCORE_POSITION),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_CHAMBER_POSITION)
        );


        BASKET_TO_CHAMBER = () -> Commands.sequence(
                Commands.runOnce(() -> claw.setState(ClawConstants.SPECIMEN_SCORING_STATE)),
                Commands.runOnce(() -> arm.setState(Arm.ArmState.SCORING_SPECIMEN)),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_CHAMBER_POSITION)
        );

        STOW_TO_SAMPLE_COLLECT = () -> Commands.sequence(
                Commands.runOnce(() -> claw.setState(ClawConstants.SAMPLE_COLLECTING_STATE)),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_SAMPLE_COLLECT_POSITION),
                Commands.runOnce(() -> arm.setState(Arm.ArmState.COLLECTING_SAMPLE))
        );

        STOW_TO_SPECIMEN_COLLECT = () -> Commands.sequence(
                Commands.runOnce(() -> claw.setState(ClawConstants.SPECIMEN_COLLECT_STATE)),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_SPECIMEN_COLLECT_POSITION),
                Commands.runOnce(() -> arm.setState(Arm.ArmState.COLLECTING_SPECIMEN))
        );

        CHAMBER_TO_SPECIMEN_COLLECT = () -> Commands.sequence(
                new MoveSlideCommand(() -> ArmConstants.SLIDE_REST_POSITION),
                new MovePivotCommand(() -> ArmConstants.PIVOT_REST_POSITION),
                Commands.runOnce(() -> claw.setState(ClawConstants.SPECIMEN_COLLECT_STATE)),
                Commands.runOnce(() -> arm.setState(Arm.ArmState.COLLECTING_SPECIMEN)),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_SPECIMEN_COLLECT_POSITION)
        );

        STOW_TO_CHAMBER = () -> Commands.sequence(
                Commands.runOnce(() -> arm.setState(Arm.ArmState.SCORING_SPECIMEN)),
                Commands.runOnce(() -> claw.setState(ClawConstants.SPECIMEN_SCORING_STATE)),
                new MovePivotCommand(() -> ArmConstants.PIVOT_SCORE_POSITION),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_CHAMBER_POSITION)
        );

        BASKET_TO_AUTO_ASCENT = () -> Commands.sequence(
                Commands.runOnce(() -> claw.setState(ClawConstants.REST_STATE)),
                new MovePivotCommand(() -> ArmConstants.PIVOT_SCORE_POSITION),
                new MoveSlideCommand(() -> 900)
        );

        GRAB = () -> Commands.sequence(
                Commands.runOnce(claw::closeClaw),
                Commands.waitMillis(ClawConstants.GRAB_DELAY)
        );

        RELEASE = () -> Commands.sequence(
                Commands.runOnce(claw::openClaw),
                Commands.waitMillis(ClawConstants.GRAB_DELAY)
        );

        SCORE_SPECIMEN = () -> Commands.sequence(
                Commands.runOnce(() -> claw.setState(ClawConstants.SPECIMEN_SCORE_STATE)),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_CHAMBER_POSITION - ArmConstants.SLIDE_CHAMBER_SCORE_OFFSET),
                Commands.waitMillis(200),
                Commands.runOnce(claw::openClaw),
                Commands.waitMillis(ClawConstants.GRAB_DELAY)
        );

        COLLECT_SAMPLE = () -> Commands.sequence(
                Commands.runOnce(() -> arm.setSlidePos(arm.getSlidePos())),
                Commands.runOnce(() -> arm.setState(Arm.ArmState.STOW)),
                Commands.runOnce(() -> claw.setWrist(limelight.getServoPos())),
                Commands.waitMillis(175),
                Commands.runOnce(() -> claw.setJointOne(ClawConstants.SAMPLE_COLLECT_JOINT_ONE_POS)),
                Commands.waitMillis(ClawConstants.PIVOT_DELAY),
                Commands.runOnce(() -> claw.setJointTwo(ClawConstants.SAMPLE_COLLECT_JOINT_TWO_POS)),
                Commands.waitMillis(ClawConstants.GRAB_DELAY)
        );

        MISSED_SEQUENCE = () -> Commands.sequence(
                Commands.runOnce(() -> claw.setState(ClawConstants.SAMPLE_COLLECTING_STATE), claw),
                Commands.waitMillis(175),
                Commands.runOnce(() -> arm.setState(Arm.ArmState.COLLECTING_SAMPLE), arm)
        );

    }

    static {
        Arm arm = Arm.getInstance();
        Claw claw = Claw.getInstance();
        LLVision llVision = LLVision.getInstance();

        TO_STOW = Commands.deferredProxy(() -> {
            switch (arm.getState()) {
                case COLLECTING_SAMPLE:
                    return Commands.defer(SAMPLE_COLLECT_TO_STOW, arm);
                case SCORING_SPECIMEN:
                    return Commands.defer(CHAMBER_TO_STOW, arm);
                case SCORING_SAMPLE:
                    return Commands.defer(BASKET_TO_STOW, arm);
                case COLLECTING_SPECIMEN:
                    return Commands.defer(SPECIMEN_COLLECT_TO_STOW, arm);
                default:
                    return Commands.none();
            }
        }).andThen(new SlideZeroCommand());

        TO_BASKET = Commands.deferredProxy(() -> {
            if (arm.getScoreType() == Arm.ScoreType.SPECIMEN) return Commands.none();
            switch (arm.getState()) {
                case STOW:
                    return Commands.defer(STOW_TO_BASKET, arm);
                case SCORING_SPECIMEN:
                    return Commands.defer(CHAMBER_TO_BASKET, arm);
                case SCORING_SAMPLE:
                    return new MovePivotCommand(() -> ArmConstants.PIVOT_SCORE_POSITION)
                            .andThen(new MoveSlideCommand(() -> ArmConstants.SLIDE_BASKET_POSITION));
                default:
                    return Commands.none();
            }
        });

        TO_CHAMBER = Commands.deferredProxy(() -> {
            if (arm.getScoreType() == Arm.ScoreType.SAMPLE) return Commands.none();
            switch (arm.getState()) {
                case COLLECTING_SPECIMEN:
                    return Commands.defer(SPECIMEN_COLLECT_TO_CHAMBER, arm);
                case SCORING_SAMPLE:
                    return Commands.defer(BASKET_TO_CHAMBER, arm);
                case STOW:
                    return Commands.defer(STOW_TO_CHAMBER, arm);
                default:
                    return Commands.none();
            }
        });

        TO_COLLECT = Commands.deferredProxy(() -> {
            if (arm.getState() == Arm.ArmState.STOW) {
                if (arm.getScoreType() == Arm.ScoreType.SPECIMEN) {
                    return Commands.defer(STOW_TO_SPECIMEN_COLLECT, arm);
                } else {
                    return Commands.defer(STOW_TO_SAMPLE_COLLECT, arm);
                }
            } else return Commands.none();
        });

        CLOSE_COLLECT = Commands.deferredProxy(() -> {
            if (arm.getState() == Arm.ArmState.STOW) {
                return Commands.sequence(
                    Commands.runOnce(() -> claw.setState(ClawConstants.SAMPLE_COLLECTING_STATE)),
                    new MoveSlideCommand(() -> ArmConstants.SLIDE_CLOSE_SAMPLE_COLLECT_POSITION),
                    Commands.runOnce(() -> arm.setState(Arm.ArmState.COLLECTING_SAMPLE))
                );
            } else return Commands.none();
        });

        ARM_ACTION = Commands.deferredProxy(() -> {
            switch (arm.getState()) {
                case COLLECTING_SAMPLE:
                    return Commands.sequence(
                            Commands.defer(COLLECT_SAMPLE, claw),
                            Commands.runOnce(llVision::setWhite),
                            Commands.defer(GRAB, claw),
                            Commands.waitMillis(100),
                            Commands.either(
                                    // If claw is in sight (missed collection)
                                    Commands.defer(MISSED_SEQUENCE, claw),
                                    // If claw is out of sight (collected)
                                    Commands.defer(SAMPLE_COLLECT_TO_STOW, arm),
                                    // Condition
                                    llVision::clawInView
                            ),
                            Commands.runOnce(llVision::setPipeline)
                    );
                case COLLECTING_SPECIMEN:
                    return Commands.sequence(
                            Commands.defer(GRAB, claw),
                            Commands.defer(SPECIMEN_COLLECT_TO_CHAMBER, arm)
                    );
                case SCORING_SAMPLE:
                    return Commands.sequence(
                            Commands.defer(RELEASE, claw)
//                            Commands.defer(BASKET_TO_STOW, arm.get())
                    );
                case SCORING_SPECIMEN:
                    return Commands.sequence(
                            Commands.defer(SCORE_SPECIMEN, arm, claw),
                            Commands.defer(CHAMBER_TO_STOW, arm)
                    );
                case STOW:
                    return Commands.either(
                            Commands.defer(STOW_TO_SAMPLE_COLLECT),
                            Commands.defer(STOW_TO_SPECIMEN_COLLECT),
                            () -> arm.getScoreType() == Arm.ScoreType.SAMPLE
                    );
                default:
                    return Commands.none();
            }
        });
    }
}
