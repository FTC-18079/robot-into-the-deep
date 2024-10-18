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
import org.firstinspires.ftc.teamcode.util.RobotGlobal;
import org.firstinspires.ftc.teamcode.util.hardware.SuccessMotor;


public class Collector extends SubsystemBase {
    Telemetry telemetry;

    // Hardware
    SuccessMotor leftSlide;
    SuccessMotor rightSlide;
//    PhotonServo deploy;
//    PhotonServo pivot;
//    PhotonServo intake;
    Servo deploy;
    Servo pivot;
    Servo intake;

    // Control loop
    double targetPose = 0;
    double output = 0;
    double lastOutput = 0.0;
    double sampleDistance = 0.0;
    PIDFController pidfController = new PIDFController(CollectorConstants.kP, CollectorConstants.kI, CollectorConstants.kD, 0.0);

    // States
    SampleColor targetColor;

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
        leftSlide = new SuccessMotor(RobotMap.getInstance().LEFT_SLIDE);
        rightSlide = new SuccessMotor(RobotMap.getInstance().RIGHT_SLIDE);

        deploy = RobotMap.getInstance().DEPLOY;
        pivot = RobotMap.getInstance().PIVOT;
        intake = RobotMap.getInstance().INTAKE;

        telemetry = RobotCore.getTelemetry();

        targetColor = SampleColor.YELLOW;
        setUpMotors();
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

    // Pivot control
    public void setPivot(double position) {
        pivot.setPosition(position);
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

    // Color states
    public void toggleTargetColor() {
        if (targetColor == SampleColor.YELLOW) {
            targetColor = (RobotGlobal.alliance == RobotGlobal.Alliance.RED) ? SampleColor.RED : SampleColor.BLUE;
        } else targetColor = SampleColor.YELLOW;
    }

    public SampleColor getTargetColor() {
        return targetColor;
    }

    @Override
    public void periodic() {
        lastOutput = output;
        output = pidfController.calculate(rightSlide.getCurrentPosition(), targetPose);
        double deltaV = output - lastOutput;

        // Limit acceleration, but not deceleration.
        // This is done to prevent belt skipping. Deceleration is ignored since the PID loop handles that
        if (Math.abs(deltaV) > CollectorConstants.MAX_DELTAV && Math.signum(output) == Math.signum(deltaV)) output = Math.signum(deltaV) * CollectorConstants.MAX_DELTAV + lastOutput;

        leftSlide.setVelocity(output);
        rightSlide.setVelocity(output);

        telemetry.addLine();
        telemetry.addData("Target Color", targetColor);
    }
}
