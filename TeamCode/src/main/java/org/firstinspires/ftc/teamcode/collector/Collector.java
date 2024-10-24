package org.firstinspires.ftc.teamcode.collector;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.util.hardware.SuccessMotor;
import org.firstinspires.ftc.teamcode.util.hardware.SuccessServo;
import org.firstinspires.ftc.teamcode.vision.LLVision;

public class Collector extends SubsystemBase {
    Telemetry telemetry;

    // Hardware
    SuccessMotor leftSlide;
    SuccessMotor rightSlide;
//    PhotonServo deploy;
//    PhotonServo pivot;
//    PhotonServo intake;
    Servo deploy;
    SuccessServo pivot;
    Servo intake;

    // Control loop
    double targetPose = 0;
    double output = 0;
    double lastOutput = 0.0;
    PIDFController pidfController = new PIDFController(CollectorConstants.kP, CollectorConstants.kI, CollectorConstants.kD, 0.0);

    CollectorState state;

    public enum CollectorState {
        STOW, COLLECTING, COLLECT, PASSTHROUGH
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
        leftSlide = new SuccessMotor(RobotMap.getInstance().LEFT_SLIDE);
        rightSlide = new SuccessMotor(RobotMap.getInstance().RIGHT_SLIDE);

        deploy = RobotMap.getInstance().DEPLOY;
        pivot = new SuccessServo(RobotMap.getInstance().PIVOT);
        intake = RobotMap.getInstance().INTAKE;

        telemetry = RobotCore.getTelemetry();

        setUpMotors();
        setState(CollectorState.STOW);
    }

    public void setUpMotors() {
        rightSlide.setVelocityThreshold(CollectorConstants.VELOCITY_THRESHOLD);
        leftSlide.setVelocityThreshold(CollectorConstants.VELOCITY_THRESHOLD);

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
    public double getTargetPose() {
        return targetPose;
    }

    public int getCurrentPosition() {
        return rightSlide.getCurrentPosition();
    }

    public double getCurrentVelocity() {
        return rightSlide.getVelocity();
    }

    public CollectorState getState() {
        return state;
    }

    // SETTERS

    public void setState(CollectorState state) {
        this.state = state;
    }

    // Extension control
    public void toRest() {
        setTargetPose(CollectorConstants.SLIDE_STOW_POS);
    }

    public void toPassthrough() {
        setTargetPose(CollectorConstants.SLIDE_PASSTHROUGH_POS);
    }

    public void setTargetPose(double position) {
        targetPose = position;
    }

    public boolean atSetPoint() {
        return Math.abs(targetPose - getCurrentPosition()) < CollectorConstants.ERROR_TOLERANCE;
    }

    // Intake control
    public void grab() {
        intake.setPosition(CollectorConstants.INTAKE_GRAB_POS);
    }

    public void release() {
        intake.setPosition(CollectorConstants.INTAKE_RELEASE_POS);
    }

    // Deploy control
    public void deploySeek() {
        deploy.setPosition(CollectorConstants.DEPLOY_SEEKING_POS);
    }

    public void deployCollect() {
        deploy.setPosition(CollectorConstants.DEPLOY_COLLECT_POS);
    }

    public void deployStow() {
        deploy.setPosition(CollectorConstants.DEPLOY_STOW_POS);
    }

    public void toCollect() {
        double pos = getCurrentPosition();
        targetPose = pos - CollectorConstants.SLIDE_COLLECT_DISPLACEMENT;
    }

    public void toTeleop() {
        targetPose = CollectorConstants.SLIDE_STOW_POS;
        lastOutput = 0;
        output = 0;
        state = CollectorState.STOW;
    }

    public void stateMachine() {
        switch (state) {
            case STOW:
            case PASSTHROUGH:
                pivot.setPosition(CollectorConstants.PIVOT_PASSTHROUGH_POS);
                break;
            case COLLECTING:
                targetPose -= LLVision.getInstance().getSampleTy();
                targetPose = MathFunctions.clamp(targetPose, 700, CollectorConstants.SLIDE_MAX_POS);
                pivot.setPosition(LLVision.getInstance().getServoPos());
                break;
            case COLLECT:
                break;
        }
    }

    public double getOutput() {
        return output;
    }

    @Override
    public void periodic() {
        lastOutput = output;
        stateMachine();
        output = pidfController.calculate(getCurrentPosition(), targetPose);

        // Limit acceleration, but not deceleration.
        // This is done to prevent belt skipping. Deceleration is ignored since the PID loop handles that
        double deltaV = output - lastOutput;
        if (Math.abs(deltaV) > CollectorConstants.MAX_DELTAV && Math.signum(output) == Math.signum(deltaV)) output = Math.signum(deltaV) * CollectorConstants.MAX_DELTAV + lastOutput;

        if (atSetPoint()) output = 0.0;

        leftSlide.setVelocity(output);
        rightSlide.setVelocity(output);

        telemetry.addLine();
        telemetry.addData("Collector state", state);
        telemetry.addData("Collector pos", getCurrentPosition());
        telemetry.addData("COLLECTOR POS", getCurrentPosition());
        telemetry.addData("COLLECTOR TARGET POS", getTargetPose());
        telemetry.addData("COLLECTOR OUTPUT", getOutput());
    }
}
