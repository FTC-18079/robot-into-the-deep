package org.firstinspires.ftc.teamcode.collector;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.RobotMap;


public class Collector extends SubsystemBase {
    Telemetry telemetry;

    // Hardware
    DcMotorEx leftSlide;
    DcMotorEx rightSlide;
    CRServo intake;
    Servo deploy;
    RevColorSensorV3 colorSensor;

    // Control loop
    double targetPose = 0;
    double output = 0;
    double lastOutput = 0.0;
    PIDFController pidfController = new PIDFController(CollectorConstants.kP, CollectorConstants.kI, CollectorConstants.kD, 0.0);

    // States
    CollectorState collectorState;
    IntakeState intakeState;
    SampleColor targetColor;

    public enum CollectorState {
        INACTIVE, SEEKING, COLLECTING
    }

    public enum IntakeState {
        INACTIVE, COLLECTING, EJECTING
    }

    public enum SampleColor {
        NONE, YELLOW, RED, BLUE
    }

    private static Collector INSTANCE = null;

    public static Collector getInstance() {
        if (INSTANCE == null) INSTANCE = new Collector();
        return INSTANCE;
    }

    private Collector() {
        leftSlide = RobotMap.getInstance().LEFT_SLIDE;
        rightSlide = RobotMap.getInstance().RIGHT_SLIDE;

        intake = RobotMap.getInstance().INTAKE;
        deploy = RobotMap.getInstance().DEPLOY;

        colorSensor = RobotMap.getInstance().COLOR_SENSOR;

        telemetry = RobotCore.getTelemetry();

        setCollectorState(CollectorState.INACTIVE);
        setUpMotors();
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

    public void setCollectorState(CollectorState collectorState) {
        this.collectorState = collectorState;
    }

    public CollectorState getCollectorState() {
        return collectorState;
    }

    public double getColor() {
        float[] hsv = new float[3];
        Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);
        return hsv[0];
    }

    public void stateMachine() {
        switch (collectorState) {
            case SEEKING:
                targetPose = CollectorConstants.MAX_SLIDE_POS * 0.80;
                intake.setPower(0.0);
                deploy.setPosition(CollectorConstants.DEPLOY_DOWN_POS);
                break;
            case COLLECTING:
                targetPose = CollectorConstants.MAX_SLIDE_POS * 0.87;
                intake.setPower(1.0);
                deploy.setPosition(CollectorConstants.DEPLOY_DOWN_POS);
                break;
            case INACTIVE:
                targetPose = CollectorConstants.MIN_SLIDE_POS;
                intake.setPower(0.0);
                deploy.setPosition(CollectorConstants.DEPLOY_STOW_POS);
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

        telemetry.addData("State", collectorState);
        telemetry.addData("DeltaV", deltaV);
        telemetry.addData("CurrentPos", getCurrentPosition());
        telemetry.addData("CurrentVel", getCurrentVelocity());
        telemetry.addData("TargetPos", targetPose);
        telemetry.addData("TargetVel", output);
    }
}
