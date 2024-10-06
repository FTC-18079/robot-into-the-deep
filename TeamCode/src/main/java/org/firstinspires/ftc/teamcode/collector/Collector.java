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
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.RobotGlobal;


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
    SampleColor targetColor;

    public enum CollectorState {
        INACTIVE, SEEKING, COLLECTING
    }

    public enum SampleColor {
        NONE(new double[]{0.0, 0.0}),
        YELLOW(CollectorConstants.YELLOW_RANGE),
        RED(CollectorConstants.RED_RANGE),
        BLUE(CollectorConstants.BLUE_RANGE);

        public final double[] range;

        private SampleColor(double[] range) {
            this.range = range;
        }
    }

    private static Collector INSTANCE = null;

    public static void resetInstance() {
        INSTANCE = null;
    }

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
        stop();
        targetColor = SampleColor.YELLOW;
        setUpMotors();
    }

    public void setUpMotors() {
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Getters
    public int getCurrentPosition() {
        return rightSlide.getCurrentPosition();
    }

    public double getCurrentVelocity() {
        return rightSlide.getVelocity();
    }

    public double getCurrentColor() {
        float[] hsv = new float[3];
        Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);
        return hsv[0];
    }

    // Collector states
    public void setCollectorState(CollectorState collectorState) {
        this.collectorState = collectorState;
    }

    public CollectorState getCollectorState() {
        return collectorState;
    }

    // Intake control
    public void in() {
        intake.setPower(1.0);
    }

    public void out() {
        intake.setPower(-1.0);
    }

    public void stop() {
        intake.setPower(0.0);
    }

    // Color states
    public void toggleTargetColor() {
        if (targetColor == SampleColor.YELLOW) {
            targetColor = (RobotGlobal.alliance == RobotGlobal.Alliance.RED) ? SampleColor.RED : SampleColor.BLUE;
        } else targetColor = SampleColor.YELLOW;
    }

    public SampleColor getTargetColor() {
        return targetColor;
    }

    // State machine
    public void stateMachine() {
        switch (collectorState) {
            case SEEKING:
                targetPose = CollectorConstants.MAX_SLIDE_POS * 0.80;
                stop();
                deploy.setPosition(CollectorConstants.DEPLOY_DOWN_POS);

                if (MathUtil.inRange(getCurrentColor(), targetColor.range[0], targetColor.range[1])) {
                    setCollectorState(CollectorState.COLLECTING);
                    RobotCore.rumbleDrive(250);
                    RobotCore.rumbleManip(250);
                }

                break;
            case COLLECTING:
                targetPose = CollectorConstants.MAX_SLIDE_POS * 0.87;
                in();
                deploy.setPosition(CollectorConstants.DEPLOY_DOWN_POS);
                break;
            case INACTIVE:
                targetPose = CollectorConstants.MIN_SLIDE_POS;
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

        telemetry.addLine();
        telemetry.addData("Collector State", collectorState);
        telemetry.addData("Target Color", targetColor);
    }
}
