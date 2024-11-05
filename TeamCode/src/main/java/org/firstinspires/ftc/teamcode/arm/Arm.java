package org.firstinspires.ftc.teamcode.arm;

import static org.firstinspires.ftc.teamcode.arm.ArmConstants.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMap;

@SuppressWarnings("unused")
public class Arm extends SubsystemBase {
    Telemetry telemetry;

    DcMotorEx rightSlide;
    DcMotorEx leftSlide;
    DcMotorEx pivotEncoder;
    CRServo rightPivot;
    CRServo leftPivot;

    PIDController slidePid;
    PIDController pivotPid;
    PIDController alignmentPid;

    static double slideOffset = 0;
    static double pivotOffset = 0;

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

        rightPivot = RobotMap.getInstance().RIGHT_PIVOT;
        leftPivot = RobotMap.getInstance().LEFT_PIVOT;

        slidePid = new PIDController(SLIDE_kP, SLIDE_kI, SLIDE_kD);
        pivotPid = new PIDController(PIVOT_kP, PIVOT_kI, PIVOT_kD);
        alignmentPid = new PIDController(ALIGN_kP, ALIGN_kI, ALIGN_kD);

        resetSlideEncoder();
        resetPivotEncoder();
        setupMotors();

        state = ArmState.STOW;
        scoreType = ScoreType.SPECIMEN;
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
        return pivotEncoder.getCurrentPosition() - PIVOT_STARTING_POS;
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

    public ArmState getState() {
        return state;
    }

    public ScoreType getScoreType() {
        return scoreType;
    }

    // SETTERS

    public void setSlidePos(double pos) {
        slidePid.setSetPoint(pos);
    }

    public void setPivotPos(double pos) {
        pivotPid.setSetPoint(pos);
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
        double slideOutput = slidePid.calculate(getSlidePos());
        double slideFeedforward = SLIDE_kF * Math.sin(Math.toRadians(getPivotTarget() / PIVOT_COUNTS_PER_REVOLUTION * 360.0));

        double pivotOutput = pivotPid.calculate(getPivotPos());
        double pivotFeedforward = PIVOT_kF * Math.cos(Math.toRadians(getPivotTarget() / PIVOT_COUNTS_PER_REVOLUTION * 360.0));

        rightSlide.setPower(slideOutput + slideFeedforward);
        leftSlide.setPower(slideOutput + slideFeedforward);

        rightPivot.setPower(pivotOutput + pivotFeedforward);
        leftPivot.setPower(pivotOutput + pivotFeedforward);
    }
}
