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

    public enum State {
        STOW, COLLECTING_SAMPLE, COLLECTING_SPECIMEN, SCORING_SAMPLE, SCORING_SPECIMEN
    }

    State state;

    private static Arm INSTANCE = null;

    public static Arm getInstance() {
        return INSTANCE;
    }

    public Arm() {
        rightSlide = RobotMap.getInstance().RIGHT_SLIDE;
        leftSlide = RobotMap.getInstance().LEFT_SLIDE;
        pivotEncoder = RobotMap.getInstance().LEFT_SLIDE;
        rightPivot = RobotMap.getInstance().RIGHT_PIVOT;
        leftPivot = RobotMap.getInstance().LEFT_PIVOT;

        slidePid = new PIDController(SLIDE.kP, SLIDE.kI, SLIDE.kD);
        pivotPid = new PIDController(PIVOT.kP, PIVOT.kI, PIVOT.kD);
        alignmentPid = new PIDController(SLIDE.alignP, SLIDE.alignI, SLIDE.alignD);

        setupMotors();

        state = State.STOW;
        INSTANCE = this;
    }

    // MOTOR SETUP

    public void setupMotors() {
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        rightPivot.setDirection(DcMotorSimple.Direction.FORWARD);
        leftPivot.setDirection(DcMotorSimple.Direction.REVERSE);

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
        return pivotEncoder.getCurrentPosition() - pivotOffset;
    }

    public double getSlideTarget() {
        return slidePid.getSetPoint();
    }

    public double getPivotTarget() {
        return pivotPid.getSetPoint();
    }

    public boolean slideAtSetPoint() {
        return Math.abs(getSlidePos() - getSlideTarget()) < SLIDE.ERROR_TOLERANCE;
    }

    public boolean pivotAtSetPoint() {
        return Math.abs(getPivotPos() - getPivotTarget()) < PIVOT.ERROR_TOLERANCE;
    }

    public State getState() {
        return state;
    }

    // SETTERS

    public void setSlidePos(double pos) {
        slidePid.setSetPoint(pos - slideOffset);
    }

    public void setPivotPos(double pos) {
        pivotPid.setSetPoint(pos - pivotOffset);
    }

    public void setSlideOffset() {
        slideOffset = getSlidePos();
    }

    public void setPivotOffset() {
        pivotOffset = getPivotPos();
    }

    /**
     * For debugging and pid tuning
     */
    public void updatePid() {
        slidePid.setPID(SLIDE.kP, SLIDE.kI, SLIDE.kD);
        pivotPid.setPID(PIVOT.kP, PIVOT.kI, PIVOT.kD);
        alignmentPid.setPID(SLIDE.alignP, SLIDE.alignI, SLIDE.alignD);
    }

    // PERIODIC

    public void stateMachine() {

    }

    @Override
    public void periodic() {
        double slideOutput = slidePid.calculate(getSlidePos());
        double slideFeedforward = SLIDE.kF * Math.sin(Math.toRadians(getPivotTarget() / PIVOT.TICKS_IN_DEGREES));
        slideOutput += slideFeedforward;

        double pivotOutput = pivotPid.calculate(getPivotPos());
        double pivotFeedforward = PIVOT.kF * Math.cos(Math.toRadians(getPivotTarget() / PIVOT.TICKS_IN_DEGREES));
        pivotOutput += pivotFeedforward;
    }
}
