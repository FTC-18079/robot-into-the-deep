package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;
import java.util.List;

public class RobotMap {
    private HardwareMap hMap;
//    private ArrayList<HardwareDevice> hardwareDevices = new ArrayList<HardwareDevice>();

    // Sensors
    public SparkFunOTOS OTOS;
    public WebcamName APRILTAG_CAMERA;

    // Drive motors
    public DcMotorEx MOTOR_FL;
    public DcMotorEx MOTOR_FR;
    public DcMotorEx MOTOR_BL;
    public DcMotorEx MOTOR_BR;

    public DcMotorEx LEFT_SLIDE;
    public DcMotorEx RIGHT_SLIDE;

    private static RobotMap instance = null;

    // Returns an instance of this
    public static RobotMap getInstance() {
        // Check that no hardware is null

        /*
        boolean hasNullHardware = false;
        for (HardwareDevice h : instance.hardwareDevices) {
            if (h == null) {
                hasNullHardware = true;
                break;
            }
        }
        */

        if (instance == null /*|| hasNullHardware*/) {
            instance = new RobotMap();
        }
        return instance;
    }

    public void init(final HardwareMap hardwareMap) {
        OTOS = hardwareMap.get(SparkFunOTOS.class, "otos");
        APRILTAG_CAMERA = hardwareMap.get(WebcamName.class, "arducam");

        MOTOR_FL = hardwareMap.get(DcMotorEx.class, "leftFront");
        MOTOR_FR = hardwareMap.get(DcMotorEx.class, "rightFront");
        MOTOR_BL = hardwareMap.get(DcMotorEx.class, "leftBack");
        MOTOR_BR = hardwareMap.get(DcMotorEx.class, "rightBack");

        LEFT_SLIDE = hardwareMap.get(DcMotorEx.class, "leftSlide");
        RIGHT_SLIDE = hardwareMap.get(DcMotorEx.class, "rightSlide");

//        hardwareDevices.add(OTOS);
//        hardwareDevices.add(APRILTAG_CAMERA);
//        hardwareDevices.add(MOTOR_FL);
//        hardwareDevices.add(MOTOR_FR);
//        hardwareDevices.add(MOTOR_BL);
//        hardwareDevices.add(MOTOR_BR);

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
