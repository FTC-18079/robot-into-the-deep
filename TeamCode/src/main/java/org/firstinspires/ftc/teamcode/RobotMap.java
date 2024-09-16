package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.CRServo;
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
    public RevColorSensorV3 COLOR_SENSOR;

    // Drive motors
    public DcMotorEx MOTOR_FL;
    public DcMotorEx MOTOR_FR;
    public DcMotorEx MOTOR_BL;
    public DcMotorEx MOTOR_BR;

    // Slides
    public DcMotorEx LEFT_SLIDE;
    public DcMotorEx RIGHT_SLIDE;

    // Elevator
    public DcMotorEx ELEVATOR;

    // Scoring
    public Servo CLAW;
    public Servo BUCKET;

    // Intake
    public CRServo INTAKE;
    public Servo DEPLOY;

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
        COLOR_SENSOR = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        MOTOR_FL = hardwareMap.get(DcMotorEx.class, "leftFront");
        MOTOR_FR = hardwareMap.get(DcMotorEx.class, "rightFront");
        MOTOR_BL = hardwareMap.get(DcMotorEx.class, "leftBack");
        MOTOR_BR = hardwareMap.get(DcMotorEx.class, "rightBack");

        LEFT_SLIDE = hardwareMap.get(DcMotorEx.class, "leftSlide");
        RIGHT_SLIDE = hardwareMap.get(DcMotorEx.class, "rightSlide");

        ELEVATOR = hardwareMap.get(DcMotorEx.class, "elevator");

        CLAW = hardwareMap.get(Servo.class, "claw");
        // TODO: uncomment
//        BUCKET = hardwareMap.get(Servo.class, "bucket");

        INTAKE = hardwareMap.get(CRServo.class, "intake");
        DEPLOY = hardwareMap.get(Servo.class, "deploy");

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
