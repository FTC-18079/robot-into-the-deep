package org.firstinspires.ftc.teamcode.arm;

import static org.firstinspires.ftc.teamcode.arm.ArmConstants.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.util.hardware.SuccessCRServo;
import org.firstinspires.ftc.teamcode.vision.LLVision;

// TODO: clean up the janky zeroing
@SuppressWarnings("unused")
public class Arm extends SubsystemBase {
    Telemetry telemetry;

    DcMotorEx rightSlide;
    DcMotorEx leftSlide;
    DcMotorEx pivotEncoder;
    SuccessCRServo rightPivot;
    SuccessCRServo leftPivot;

    PIDController slidePid;
    PIDController pivotPid;
    PIDController alignmentPid;

    static double slideOffset = 0;
    static double pivotOffset = 0;

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
        rightSlide = RobotMap.getInstance().RIGHT_SLIDE;
        leftSlide = RobotMap.getInstance().LEFT_SLIDE;

        pivotEncoder = RobotMap.getInstance().MOTOR_BR;

        rightPivot = new SuccessCRServo(RobotMap.getInstance().RIGHT_PIVOT);
        leftPivot = new SuccessCRServo(RobotMap.getInstance().LEFT_PIVOT);

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

        rightPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        leftPivot.setDirection(DcMotorSimple.Direction.FORWARD);

        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetSlideEncoder() {
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetPivotEncoder() {
        pivotEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // GETTERS

    public double getSlidePos() {
        return rightSlide.getCurrentPosition() - slideOffset;
    }

    public double getPivotPos() {
        return -pivotEncoder.getCurrentPosition();
    }

    public double getSlideTarget() {
        return slidePid.getSetPoint();
    }

    public double getPivotTarget() {
        return pivotPid.getSetPoint();
    }

    public boolean slideAtSetPoint() {
        return Math.abs(slidePid.getPositionError()) < SLIDE_ERROR_TOLERANCE;
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
        rightPivot.setPower(power);
        leftPivot.setPower(power);
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

    public void biasPivotUp() {
        PIVOT_FUDGE += 0.25;
        PIVOT_SCORE_POSITION += (PIVOT_FUDGE / 360.0) * PIVOT_COUNTS_PER_REVOLUTION;

        if (state == ArmState.SCORING_SAMPLE) {
            pivotPid.setSetPoint(PIVOT_SCORE_POSITION);
        }
    }

    public void biasPivotDown() {
        PIVOT_FUDGE -= 0.25;
        PIVOT_SCORE_POSITION += (PIVOT_FUDGE / 360.0) * PIVOT_COUNTS_PER_REVOLUTION;

        if (state == ArmState.SCORING_SAMPLE) {
            pivotPid.setSetPoint(PIVOT_SCORE_POSITION);
        }
    }

    public void setSlideOffset() {
        slideOffset += getSlidePos();
    }

    public void setPivotOffset() {
        pivotOffset += getPivotPos();
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

    public void stateMachine() {

    }

    @Override
    public void periodic() {
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
        double slideFeedforward = SLIDE_kF * Math.sin(Math.toRadians(getPivotTarget() / PIVOT_COUNTS_PER_REVOLUTION * 360.0));
        if (state == ArmState.STOW) slideFeedforward = 0;

        double pivotOutput = pivotPid.calculate(getPivotPos());
        double pivotFeedforward = PIVOT_kF * Math.cos(Math.toRadians(getPivotTarget() / PIVOT_COUNTS_PER_REVOLUTION * 360.0));

        if (pivotAtSetPoint() && getPivotTarget() == PIVOT_REST_POSITION) {
            pivotOutput = 0.0;
        }

        if (!slideZeroing) rightSlide.setPower(slideOutput + slideFeedforward);
        if (!slideZeroing) leftSlide.setPower(slideOutput + slideFeedforward);

        if (!pivotZeroing) rightPivot.setPower(pivotOutput + pivotFeedforward);
        if (!pivotZeroing) leftPivot.setPower(pivotOutput + pivotFeedforward);
    }
}
