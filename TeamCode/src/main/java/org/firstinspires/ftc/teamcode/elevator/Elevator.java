package org.firstinspires.ftc.teamcode.elevator;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.util.hardware.SuccessMotor;

public class Elevator extends SubsystemBase {
    Telemetry telemetry;

    // Hardware
    SuccessMotor elevator;
    Servo claw;
    Servo bucket;
    Servo door;

    public enum ScoreType {
        SAMPLE, SPECIMEN
    }

    ScoreType scoreType;

    // Control loop
    PIDFController pidfController = new PIDFController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, ElevatorConstants.kF);
    double targetPos = 0.0;
    double output = 0.0;
    double lastOutput = 0.0;

    private static Elevator INSTANCE = null;

    public static void resetInstance() {
        INSTANCE = null;
    }

    public static Elevator getInstance() {
        if (INSTANCE == null) INSTANCE = new Elevator();
        return INSTANCE;
    }

    private Elevator() {
        elevator = new SuccessMotor(RobotMap.getInstance().ELEVATOR);
        claw = RobotMap.getInstance().CLAW;
        bucket = RobotMap.getInstance().BUCKET;
        door = RobotMap.getInstance().DOOR;

        scoreType = ScoreType.SPECIMEN;

        this.telemetry = RobotCore.getTelemetry();

        setupMotor();
    }

    public void setupMotor() {
        elevator.setVelocityThreshold(ElevatorConstants.VELOCITY_THRESHOLD);

        elevator.setDirection(DcMotorSimple.Direction.FORWARD);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Claw controls
    public void toggleClaw() {
        if (claw.getPosition() == ElevatorConstants.CLAW_GRAB_POS) openClaw();
        else closeClaw();
    }

    public void openClaw() {
        claw.setPosition(ElevatorConstants.CLAW_OPEN_POS);
    }

    public void closeClaw() {
        claw.setPosition(ElevatorConstants.CLAW_GRAB_POS);
    }

    // Game piece types
    public void setScoreType(ScoreType scoreType) {
        this.scoreType = scoreType;
    }

    public void toggleScoreType() {
        if (scoreType == ScoreType.SAMPLE) setScoreType(ScoreType.SPECIMEN);
        else setScoreType(ScoreType.SAMPLE);
    }

    public ScoreType getScoreType() {
        return scoreType;
    }

    // Door movement
    public void closeDoor() {
        door.setPosition(ElevatorConstants.DOOR_CLOSE_POS);
    }

    public void openDoor() {
        door.setPosition(ElevatorConstants.DOOR_OPEN_POS);
    }

    // Bucket poses
    public void passthroughBucket() {
        bucket.setPosition(ElevatorConstants.BUCKET_PASSTHROUGH_POS);
    }

    public void tapBucket() {
        bucket.setPosition(ElevatorConstants.BUCKET_TAP_POS);
    }

    public void restBucket() {
        bucket.setPosition(ElevatorConstants.BUCKET_REST_POS);
    }

    public void scoreBucket() {
        bucket.setPosition(ElevatorConstants.BUCKET_SCORE_POS);
    }



    // Elevator pose setting
    public void toRest() {
        targetPos = ElevatorConstants.LIFT_POS_REST;
    }

    public void toLow() {
        if (scoreType == ScoreType.SAMPLE) targetPos = ElevatorConstants.LIFT_POS_LOW_BASKET;
        else targetPos = ElevatorConstants.LIFT_POS_HIGH_CHAMBER;
    }

    public void toHigh() {
        if (scoreType == ScoreType.SAMPLE) targetPos = ElevatorConstants.LIFT_POS_HIGH_BASKET;
        else targetPos = ElevatorConstants.LIFT_POS_HIGH_CHAMBER;
    }

    public double getTargetPos() {
        return targetPos;
    }

    public boolean atSetPoint() {
        return Math.abs(elevator.getCurrentPosition() - targetPos) < ElevatorConstants.POSITION_TOLERANCE;
    }

    public void scoreChamberHigh() {
        targetPos = ElevatorConstants.LIFT_POS_HIGH_CHAMBER_SCORE;
    }

    @Override
    public void periodic() {
        lastOutput = output;
        output = pidfController.calculate(elevator.getCurrentPosition(), targetPos);
        double deltaV = output - lastOutput;

        // Limit acceleration
        if (Math.abs(deltaV) > ElevatorConstants.MAX_DELTAV && Math.signum(output) == Math.signum(deltaV)) {
            output = Math.signum(deltaV) * ElevatorConstants.MAX_DELTAV + lastOutput;
        }

        // Supply no power if we're at zero
        if (atSetPoint() && targetPos == 0) output = 0.0;
        elevator.setVelocity(output);

        telemetry.addLine();
        telemetry.addData("Scoring Element", scoreType);
    }
}
