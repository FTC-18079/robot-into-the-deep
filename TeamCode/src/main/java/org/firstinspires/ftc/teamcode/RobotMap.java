package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class RobotMap {
    public SparkFunOTOS OTOS;
    private HardwareMap hMap;

    public DcMotorEx MOTOR_FL;
    public DcMotorEx MOTOR_FR;
    public DcMotorEx MOTOR_BL;
    public DcMotorEx MOTOR_BR;

    private static RobotMap instance = null;

    public static RobotMap getInstance(){
        if (instance == null){
            instance = new RobotMap();
        }
        return instance;
    }

    public void init(final HardwareMap hardwareMap){
        MOTOR_FL = hardwareMap.get(DcMotorEx.class, "leftFront");
        MOTOR_FR = hardwareMap.get(DcMotorEx.class, "rightFront");
        MOTOR_BL = hardwareMap.get(DcMotorEx.class, "leftBack");
        MOTOR_BR = hardwareMap.get(DcMotorEx.class, "rightBack");

        OTOS = hardwareMap.get(SparkFunOTOS.class, "otos");

        this.hMap = hardwareMap;
    }

    public List<LynxModule> getLynxModules() {
        return hMap.getAll(LynxModule.class);
    }

    public HardwareMap getHardwareMap() {
        return this.hMap;
    }
}
