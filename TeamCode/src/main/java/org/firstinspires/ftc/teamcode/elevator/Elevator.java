package org.firstinspires.ftc.teamcode.elevator;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.RobotMap;

public class Elevator extends SubsystemBase {
    Telemetry telemetry;

    // Hardware
    DcMotorEx elevator;
    Servo claw;
    Servo bucket;

    // Control loop
    PIDFController pidfController = new PIDFController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, 0.0);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ElevatorConstants.kS, ElevatorConstants.kV, ElevatorConstants.kA);

    private static Elevator INSTANCE = null;

    public static void resetInstance() {
        INSTANCE = null;
    }

    public static Elevator getInstance() {
        if (INSTANCE == null) INSTANCE = new Elevator();
        return INSTANCE;
    }

    private Elevator() {
        elevator = RobotMap.getInstance().ELEVATOR;
        claw = RobotMap.getInstance().CLAW;
//        bucket = RobotMap.getInstance().BUCKET;

        this.telemetry = RobotCore.getTelemetry();

        setupMotor();
    }

    public void setupMotor() {
        elevator.setDirection(DcMotorSimple.Direction.FORWARD);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void periodic() {
        telemetry.addLine();
    }
}
