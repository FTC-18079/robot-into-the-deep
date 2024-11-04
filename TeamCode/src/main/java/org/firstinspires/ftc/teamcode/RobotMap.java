package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

// TODO: change drive motors back to photon motors
public class RobotMap {
    private HardwareMap hMap;

    // Sensors
    public SparkFunOTOS OTOS;
    public WebcamName APRILTAG_CAMERA;
    public Limelight3A LIMELIGHT;

    // Drive motors
    public DcMotorEx MOTOR_FL;
    public DcMotorEx MOTOR_FR;
    public DcMotorEx MOTOR_BL;
    public DcMotorEx MOTOR_BR;

    // Slides
    public DcMotorEx LEFT_SLIDE;
    public DcMotorEx RIGHT_SLIDE;

    // Pivot
    public CRServo LEFT_PIVOT;
    public CRServo RIGHT_PIVOT;

    // Elevator
    public DcMotorEx ELEVATOR;

    // Scoring
    public Servo CLAW;
    public Servo BUCKET;
    public Servo DOOR;

    // Collector
//    public PhotonServo DEPLOY;
//    public PhotonServo PIVOT;
//    public PhotonServo INTAKE;
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
        this.hMap = hardwareMap;

        OTOS = hardwareMap.get(SparkFunOTOS.class, "otos");
//        APRILTAG_CAMERA = hardwareMap.get(WebcamName.class, "arducam");
        LIMELIGHT = hardwareMap.get(Limelight3A.class, "limelight");

        MOTOR_FL = hardwareMap.get(DcMotorEx.class, "leftFront");
        MOTOR_FR = hardwareMap.get(DcMotorEx.class, "rightFront");
        MOTOR_BL = hardwareMap.get(DcMotorEx.class, "leftBack");
        MOTOR_BR = hardwareMap.get(DcMotorEx.class, "rightBack");

        LEFT_SLIDE = hardwareMap.get(DcMotorEx.class, "leftSlide");
        RIGHT_SLIDE = hardwareMap.get(DcMotorEx.class, "rightSlide");

        RIGHT_PIVOT = hardwareMap.get(CRServo.class, "rightPivot");
        LEFT_PIVOT = hardwareMap.get(CRServo.class, "leftPivot");

        for (LynxModule hub : getLynxModules()) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
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
