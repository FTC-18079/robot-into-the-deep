package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;

import java.util.ArrayList;
import java.util.List;

public class RobotMap {
    private HardwareMap hMap;
    private final List<HardwareDevice> devices = new ArrayList<>();

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
    public CRServo PIVOT_FL;
    public CRServo PIVOT_FR;
    public CRServo PIVOT_BL;
    public CRServo PIVOT_BR;
    public AnalogInput PIVOT_ENCODER;

    // Claw
    public Servo CLAW;
    public Servo WRIST;
    public Servo JOINT_ONE;
    public Servo JOINT_TWO;

    // Climb
    public DcMotorEx CLIMB_MOTOR;

    private static RobotMap instance = null;

    // Returns an instance of this
    public static RobotMap getInstance() {
        if (instance == null) {
            instance = new RobotMap();
        }
        return instance;
    }

    public void init(final HardwareMap hardwareMap) {
        devices.clear();

        this.hMap = hardwareMap;

        OTOS = hardwareMap.get(SparkFunOTOS.class, "otos");
//        APRILTAG_CAMERA = hardwareMap.get(WebcamName.class, "arducam");
        LIMELIGHT = hardwareMap.get(Limelight3A.class, "limelight");

        MOTOR_FL = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftFrontMotorName);
        MOTOR_FR = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightFrontMotorName);
        MOTOR_BL = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftRearMotorName);
        MOTOR_BR = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightRearMotorName);

        LEFT_SLIDE = hardwareMap.get(DcMotorEx.class, "leftSlide");
        RIGHT_SLIDE = hardwareMap.get(DcMotorEx.class, "rightSlide");

        PIVOT_FR = hardwareMap.get(CRServo.class, "pivotFR");
        PIVOT_FL = hardwareMap.get(CRServo.class, "pivotFL");
        PIVOT_BL = hardwareMap.get(CRServo.class, "pivotBL");
        PIVOT_BR = hardwareMap.get(CRServo.class, "pivotBR");
        PIVOT_ENCODER = hardwareMap.get(AnalogInput.class, "pivotEncoder");

        CLAW = hardwareMap.get(Servo.class, "claw");
        WRIST = hardwareMap.get(Servo.class, "wrist");
        JOINT_ONE = hardwareMap.get(Servo.class, "jointOne");
        JOINT_TWO = hardwareMap.get(Servo.class, "jointTwo");

        CLIMB_MOTOR = hardwareMap.get(DcMotorEx.class, "climb");

        addDevices();
    }

    private void addDevices() {
        devices.add(getInstance().OTOS);
        devices.add(getInstance().LIMELIGHT);
        devices.add(getInstance().MOTOR_FL);
        devices.add(getInstance().MOTOR_FR);
        devices.add(getInstance().MOTOR_BL);
        devices.add(getInstance().MOTOR_BR);
        devices.add(getInstance().LEFT_SLIDE);
        devices.add(getInstance().RIGHT_SLIDE);
        devices.add(getInstance().PIVOT_FR);
        devices.add(getInstance().PIVOT_FL);
        devices.add(getInstance().PIVOT_BL);
        devices.add(getInstance().PIVOT_BR);
        devices.add(getInstance().PIVOT_ENCODER);
        devices.add(getInstance().CLAW);
        devices.add(getInstance().WRIST);
        devices.add(getInstance().JOINT_ONE);
        devices.add(getInstance().JOINT_TWO);
        devices.add(getInstance().CLIMB_MOTOR);
    }

    public List<HardwareDevice> getDevices() {
        return devices;
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
