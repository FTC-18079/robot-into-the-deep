package org.firstinspires.ftc.teamcode.collector;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.RobotMap;


public class Collector extends SubsystemBase {
    DcMotorEx leftSlide;
    DcMotorEx rightSlide;
    Telemetry telemetry;

    double targetPose = 0;
    double output = 0;
    double lastOutput = 0.0;

    CollectorState state;

    PIDFController pidfController = new PIDFController(CollectorConstants.kP, CollectorConstants.kI, CollectorConstants.kD, 0.0);

    private static Collector INSTANCE;

    private Collector() {
        leftSlide = RobotMap.getInstance().LEFT_SLIDE;
        rightSlide = RobotMap.getInstance().RIGHT_SLIDE;
        telemetry = RobotCore.getTelemetry();

        setState(CollectorState.INACTIVE);
        setUpMotors();
    }

    public static Collector getInstance() {
        if (INSTANCE == null) INSTANCE = new Collector();
        return INSTANCE;
    }

    public void setUpMotors() {
        pidfController.setPIDF(CollectorConstants.kP, CollectorConstants.kI, CollectorConstants.kD, 0.0);

        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int getCurrentPosition() {
        return rightSlide.getCurrentPosition();
    }

    public double getCurrentVelocity() {
        return rightSlide.getVelocity();
    }

    public enum CollectorState {
        INACTIVE, SEEKING, COLLECTING
    }

    public void setState(CollectorState state) {
        this.state = state;
    }

    public void stateMachine() {
        switch (state) {
            case SEEKING:
                targetPose = CollectorConstants.MAX_SLIDE_POS * 0.80;
                break;
            case COLLECTING:
                targetPose = CollectorConstants.MAX_SLIDE_POS * 0.90;
                break;
            case INACTIVE:
                targetPose = CollectorConstants.MIN_SLIDE_POS;
                break;
        }
    }

    @Override
    public void periodic() {
        stateMachine();

        lastOutput = output;
        output = pidfController.calculate(rightSlide.getCurrentPosition(), targetPose);
        double deltaV = output - lastOutput;

        // Limit acceleration, but not deceleration.
        // This is done to prevent belt skipping. Deceleration is ignored since the PID loop handles that
        if (Math.abs(deltaV) > CollectorConstants.MAX_DELTAV && Math.signum(output) == Math.signum(deltaV)) output = Math.signum(deltaV) * CollectorConstants.MAX_DELTAV + lastOutput;
        if (Math.abs(output) < CollectorConstants.MIN_VELOCITY || Math.abs(targetPose - getCurrentPosition()) <= CollectorConstants.ERROR_TOLERANCE) output = 0.0;

        leftSlide.setVelocity(output);
        rightSlide.setVelocity(output);
        telemetry.addData("State", state);
        telemetry.addData("DeltaV", deltaV);
        telemetry.addData("CurrentPos", getCurrentPosition());
        telemetry.addData("CurrentVel", getCurrentVelocity());
        telemetry.addData("TargetPos", targetPose);
        telemetry.addData("TargetVel", output);
    }
}
