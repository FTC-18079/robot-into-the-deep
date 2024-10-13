package org.firstinspires.ftc.teamcode;

import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

public class RobotMap {
    private HardwareMap hMap;

    // Sensors
    public SparkFunOTOS OTOS;
    public WebcamName APRILTAG_CAMERA;

    // Drive motors
    public PhotonDcMotor MOTOR_FL;
    public PhotonDcMotor MOTOR_FR;
    public PhotonDcMotor MOTOR_BL;
    public PhotonDcMotor MOTOR_BR;

    // Slides
    public DcMotorEx LEFT_SLIDE;
    public DcMotorEx RIGHT_SLIDE;

    // Elevator
    public DcMotorEx ELEVATOR;

    // Scoring
    public Servo CLAW;
    public Servo BUCKET;
    public Servo DOOR;

    // Collector
    public Servo DEPLOY;
    public Servo PIVOT;
    public Servo INTAKE;

    private static RobotMap instance = null;

    // Returns an instance of this
    public static RobotMap getInstance() {
        if (instance == null) {
            instance = new RobotMap();
        }
        return instance;
    }

    public void init(final HardwareMap hardwareMap) {
        OTOS = hardwareMap.get(SparkFunOTOS.class, "otos");
        APRILTAG_CAMERA = hardwareMap.get(WebcamName.class, "arducam");

        MOTOR_FL = (PhotonDcMotor) hardwareMap.dcMotor.get("leftFront");
        MOTOR_FR = (PhotonDcMotor) hardwareMap.dcMotor.get("rightFront");
        MOTOR_BL = (PhotonDcMotor) hardwareMap.dcMotor.get("leftBack");
        MOTOR_BR = (PhotonDcMotor) hardwareMap.dcMotor.get("rightBack");

        LEFT_SLIDE = hardwareMap.get(DcMotorEx.class, "leftSlide");
        RIGHT_SLIDE = hardwareMap.get(DcMotorEx.class, "rightSlide");

        ELEVATOR = hardwareMap.get(DcMotorEx.class, "elevator");

        CLAW = hardwareMap.get(Servo.class, "claw");
        BUCKET = hardwareMap.get(Servo.class, "bucket");
        DOOR = hardwareMap.get(Servo.class, "door");

        DEPLOY = hardwareMap.get(Servo.class, "deploy");
        PIVOT = hardwareMap.get(Servo.class, "pivot");
        INTAKE = hardwareMap.get(Servo.class, "intake");

        this.hMap = hardwareMap;
    }

    // Get hubs
    public List<LynxModule> getLynxModules() {
        return hMap.getAll(LynxModule.class);
    }

    // Get hardwareMap instance
    public HardwareMap getHardwareMap() {
        return this.hMap;
    }
}
