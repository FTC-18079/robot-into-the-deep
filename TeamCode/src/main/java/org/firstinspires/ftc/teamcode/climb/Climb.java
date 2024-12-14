package org.firstinspires.ftc.teamcode.climb;

import static org.firstinspires.ftc.teamcode.climb.ClimbConstants.*;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hydra;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.RobotStatus;
import org.firstinspires.ftc.teamcode.util.SubsystemIF;
import org.firstinspires.ftc.teamcode.util.hardware.SuccessMotor;

public class Climb extends SubsystemIF {
    Telemetry telemetry;

    SuccessMotor climbMotor;

    PIDController climbPid;

    private final static Climb INSTANCE = new Climb();

    public static Climb getInstance() {
        return INSTANCE;
    }

    // INITIALIZE

    private Climb() {
        climbPid = new PIDController(kP, kI, kD);
        climbPid.setSetPoint(0);
    }

    @Override
    public void onAutonomousInit() {
        telemetry = Hydra.getInstance().getTelemetry();
        configureHardware();
        resetClimbEncoder();
        climbPid.setSetPoint(getClimbPos());
    }

    @Override
    public void onTeleopInit() {
        telemetry = Hydra.getInstance().getTelemetry();
        configureHardware();
        climbPid.setSetPoint(getClimbPos());
    }

    // MOTOR SETUP

    public void configureHardware() {
        climbMotor = new SuccessMotor(RobotMap.getInstance().CLIMB_MOTOR);

        climbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climbMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        climbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetClimbEncoder() {
        climbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // GETTERS

    public double getClimbPos() {
        return climbMotor.getCurrentPosition();
    }

    public double getClimbTarget() {
        return climbPid.getSetPoint();
    }

    public boolean climbAtSetPoint() {
        return Math.abs(getClimbPos() - getClimbTarget()) < CLIMB_ERROR_TOLERANCE;
    }

    // SETTERS

    public void setFloat() {
        climbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setBrake() {
        climbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power) {
        climbMotor.setPower(power);
    }

    public void setClimbPos(double pos) {
        climbPid.setSetPoint(pos);
    }

    // PERIODIC

    @Override
    public void periodic() {
        telemetry.addLine();
        telemetry.addData("Climb Position", getClimbPos());
        telemetry.addData("Climb State", RobotStatus.climbState);
    }
}
