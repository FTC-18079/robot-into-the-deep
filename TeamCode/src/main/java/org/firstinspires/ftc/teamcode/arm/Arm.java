package org.firstinspires.ftc.teamcode.arm;

import static org.firstinspires.ftc.teamcode.arm.ArmConstants.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.util.hardware.SuccessMotor;
import org.firstinspires.ftc.teamcode.vision.LLVision;

// TODO: clean up the janky zeroing
public class Arm extends SubsystemBase {
    Telemetry telemetry;

    SuccessMotor rightSlide;
    SuccessMotor leftSlide;
    Pivot pivot;

    PIDController slidePid;
    PIDController pivotPid;
    PIDController alignmentPid;

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

    private static Arm INSTANCE = null;

    public static Arm getInstance() {
        return INSTANCE;
    }

    public Arm() {
        rightSlide = new SuccessMotor(RobotMap.getInstance().RIGHT_SLIDE);
        leftSlide = new SuccessMotor(RobotMap.getInstance().LEFT_SLIDE);

        pivot = new Pivot();

        slidePid = new PIDController(SLIDE_kP, SLIDE_kI, SLIDE_kD);
        pivotPid = new PIDController(PIVOT_kP, PIVOT_kI, PIVOT_kD);
        alignmentPid = new PIDController(ALIGN_kP, ALIGN_kI, ALIGN_kD);
        alignmentPid.setSetPoint(0.0);

        resetSlideEncoder();
        resetPivotEncoder();
        setupMotors();

        state = ArmState.STOW;
        scoreType = ScoreType.SAMPLE;
        telemetry = RobotCore.getTelemetry();
        INSTANCE = this;
    }

    // MOTOR SETUP

    public void setupMotors() {
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

    // TODO: either remove or figure out how to make offsets work
    public void resetPivotEncoder() {

    }

    // GETTERS

    public double getSlidePos() {
        return leftSlide.getCurrentPosition();
    }

    public double getPivotPos() {
        return pivot.getPosition();
    }

    public double getSlideTarget() {
        return slidePid.getSetPoint();
    }

    public double getPivotTarget() {
        return pivotPid.getSetPoint();
    }

    public boolean slideAtSetPoint() {
        return Math.abs(getPivotTarget() - getPivotPos()) < SLIDE_ERROR_TOLERANCE;
    }

    public boolean pivotAtSetPoint() {
        return Math.abs(getPivotPos() - getPivotTarget()) < PIVOT_ERROR_TOLERANCE;
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
        // temp
        updatePid();

        telemetry.addData("Arm State", state);
        telemetry.addData("Scoring Piece", scoreType);
        telemetry.addData("Pivot target", getPivotTarget());
        telemetry.addData("Pivot pos", getPivotPos());
        telemetry.addData("Slide target", getSlideTarget());
        telemetry.addData("Slide pos", getSlidePos());

        double slideOutput = slidePid.calculate(getSlidePos());
        if (state == ArmState.COLLECTING_SAMPLE) {
            double ty = LLVision.getInstance().getSampleTy();
            slideOutput = alignmentPid.calculate(ty);
            if (slideOutput > 0 && getSlidePos() >= SLIDE_SAMPLE_COLLECT_POSITION + 50) slideOutput = 0.0;
            if (Math.abs(ty) < ALIGN_ERROR_TOLERANCE) slideOutput = 0.0;
        }
        double slideFeedforward = SLIDE_kF * Math.sin(Math.toRadians(getPivotTarget()));
        if (state == ArmState.STOW) slideFeedforward = 0;

        double pivotOutput = pivotPid.calculate(getPivotPos());
        double pivotFeedforward = PIVOT_kF * Math.cos(Math.toRadians(getPivotTarget()));

        if (!slideZeroing) rightSlide.setPower(slideOutput + slideFeedforward);
        if (!slideZeroing) leftSlide.setPower(slideOutput + slideFeedforward);

        if (pivotAtSetPoint() && getPivotTarget() == PIVOT_REST_POSITION) pivot.setPower(0);
        else pivot.setPower(pivotOutput + pivotFeedforward);
    }
}
