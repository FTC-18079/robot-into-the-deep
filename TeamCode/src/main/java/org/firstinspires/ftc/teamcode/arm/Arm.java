package org.firstinspires.ftc.teamcode.arm;

import static org.firstinspires.ftc.teamcode.arm.ArmConstants.*;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hydra;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.RobotStatus;
import org.firstinspires.ftc.teamcode.arm.commands.MovePivotCommand;
import org.firstinspires.ftc.teamcode.arm.commands.MoveSlideCommand;
import org.firstinspires.ftc.teamcode.arm.commands.SlideZeroCommand;
import org.firstinspires.ftc.teamcode.util.SubsystemIF;
import org.firstinspires.ftc.teamcode.util.commands.Commands;
import org.firstinspires.ftc.teamcode.util.hardware.SuccessMotor;
import org.firstinspires.ftc.teamcode.vision.LLVision;

public class Arm extends SubsystemIF {
    Telemetry telemetry;

    SuccessMotor rightSlide;
    SuccessMotor leftSlide;
    Pivot pivot;

    PIDController slidePid;
    PIDController pivotPid;
    PIDController alignmentPid;

    private double lastPivotPos;
    public boolean slideZeroing = false;
    public boolean pivotZeroing = false;

    public enum ArmState {
        STOW, COLLECTING_SAMPLE, COLLECTING_SPECIMEN, SCORING_SAMPLE, SCORING_SPECIMEN
    }

    public enum ScoreType {
        SPECIMEN, SAMPLE
    }

    ArmState state;
    ScoreType scoreType;

    private static final Arm INSTANCE = new Arm();

    public static Arm getInstance() {
        return INSTANCE;
    }

    private Arm() {
        slidePid = new PIDController(SLIDE_kP, SLIDE_kI, SLIDE_kD);
        pivotPid = new PIDController(PIVOT_kP, PIVOT_kI, PIVOT_kD);
        alignmentPid = new PIDController(ALIGN_kP, ALIGN_kI, ALIGN_kD);
        alignmentPid.setSetPoint(ALIGN_tY);

        state = ArmState.STOW;
        scoreType = ScoreType.SAMPLE;
    }

    // INITIALIZE

    @Override
    public void onAutonomousInit() {
        telemetry = Hydra.getInstance().getTelemetry();
        configureHardware();

        resetSlideEncoder();

        setPivotPos(getPivotPos());
        slidePid.setSetPoint(getSlidePos());

        lastPivotPos = pivot.getPosition();
    }

    @Override
    public void onTeleopInit() {
        telemetry = Hydra.getInstance().getTelemetry();
        configureHardware();

        setPivotPos(getPivotPos());
        slidePid.setSetPoint(getSlidePos());

        Command cmd;
        switch (RobotStatus.autoRan) {
            case SAMPLE:
                cmd = Commands.sequence(
                        Commands.waitUntil(RobotStatus::isEnabled),
                        Commands.runOnce(() -> setState(ArmState.SCORING_SAMPLE)),
                        new MoveSlideCommand(() -> SLIDE_CHAMBER_POSITION + 150)
                );
                break;
            case SPECIMEN:
                cmd = Commands.sequence(
                        Commands.waitUntil(RobotStatus::isEnabled),
                        Commands.runOnce(() -> setState(ArmState.STOW)),
                        new MovePivotCommand(() -> PIVOT_REST_POSITION)
                );
                break;
            default:
                cmd = Commands.none();
                break;
        }
        cmd.schedule();

        lastPivotPos = pivot.getPosition();
    }

    // MOTOR SETUP

    public void configureHardware() {
        rightSlide = new SuccessMotor(RobotMap.getInstance().RIGHT_SLIDE);
        leftSlide = new SuccessMotor(RobotMap.getInstance().LEFT_SLIDE);

        pivot = new Pivot();

        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetSlideEncoder() {
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void floatNeutralMode() {
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    // GETTERS

    public double getSlidePos() {
        return leftSlide.getCurrentPosition();
    }

    // Note: THIS ONLY WORKS ASSUMING THE ENCODER NEVER TURNS >180 COUNTS IN ANY SINGLE LOOP
    public double getPivotPos() {
        double position = pivot.getPosition();
        double diff = position - lastPivotPos;

        double absoluteAngle = (360 + diff) % 360;
        double num = lastPivotPos + ((absoluteAngle > 180) ? absoluteAngle - 360 : absoluteAngle);

        lastPivotPos = num;
        return num;
    }

    public double getSlideTarget() {
        return slidePid.getSetPoint();
    }

    public double getPivotTarget() {
        return pivotPid.getSetPoint();
    }

    public boolean slideAtSetPoint() {
        return Math.abs(getSlidePos() - getSlideTarget()) < SLIDE_ERROR_TOLERANCE;
    }

    public boolean pivotAtSetPoint() {
        return Math.abs(getPivotPos() - getPivotTarget()) < PIVOT_ERROR_TOLERANCE;
    }

    public double getSlideVelocity() {
        return leftSlide.getVelocity();
    }

    public ArmState getState() {
        return state;
    }

    public ScoreType getScoreType() {
        return scoreType;
    }

    // SETTERS

    public void setPivotPower(double power) {
        pivot.setPower(power);
    }

    public void setSlidePower(double power) {
        rightSlide.setPower(power);
        leftSlide.setPower(power);
    }

    public void setSlidePos(double pos) {
        slidePid.setSetPoint(pos);
    }

    public void setPivotPos(double pos) {
        pivotPid.setSetPoint(pos);
    }

    public void setState(ArmState state) {
        this.state = state;
    }

    public void setScoreType(ScoreType scoreType) {
        this.scoreType = scoreType;
    }

    /**
     * For debugging and pid tuning
     */
    public void updatePid() {
        slidePid.setPID(SLIDE_kP, SLIDE_kI, SLIDE_kD);
        pivotPid.setPID(PIVOT_kP, PIVOT_kI, PIVOT_kD);
        alignmentPid.setPID(ALIGN_kP, ALIGN_kI, ALIGN_kD);
    }

    // PERIODIC

    @Override
    public void periodic() {
        updatePid();

        telemetry.addLine();
        telemetry.addData("Arm State", state);
        telemetry.addData("Scoring Piece", scoreType);
        telemetry.addData("Pivot target", getPivotTarget());
        telemetry.addData("Pivot pos", getPivotPos());
        telemetry.addData("Slide target", getSlideTarget());
        telemetry.addData("Slide pos", getSlidePos());

        double slideOutput = slidePid.calculate(getSlidePos());
        if (state == ArmState.COLLECTING_SAMPLE) {
            double ty = LLVision.getInstance().getSampleTy();
            slideOutput = -alignmentPid.calculate(ty);
            if (slideOutput > 0 && getSlidePos() >= SLIDE_SAMPLE_COLLECT_POSITION) slideOutput = 0.0;
            if (Math.abs(ty) < ALIGN_ERROR_TOLERANCE) slideOutput = 0.0;
        } else if (slideAtSetPoint()) slideOutput = 0;
        double slideFeedforward = SLIDE_kF * Math.sin(Math.toRadians((getPivotTarget() - PIVOT_REST_POSITION) / PIVOT_GEAR_RATIO));
        if (state == ArmState.STOW) slideFeedforward = 0;

        double pivotOutput = pivotPid.calculate(getPivotPos());
        double pivotFeedforward = PIVOT_kF * Math.cos(Math.toRadians((getPivotTarget() - PIVOT_REST_POSITION) / PIVOT_GEAR_RATIO));

        if (!slideZeroing) setSlidePower(slideOutput + slideFeedforward);

        if (pivotAtSetPoint() && getPivotTarget() == PIVOT_REST_POSITION) pivot.setPower(0);
        else if (!pivotZeroing) pivot.setPower(pivotOutput + pivotFeedforward);
    }
}
