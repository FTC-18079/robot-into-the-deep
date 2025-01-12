package org.firstinspires.ftc.teamcode.util.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.RobotStatus;
import org.firstinspires.ftc.teamcode.arm.Arm;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Robot Reset", group = "Util")
public class ResetRobot extends OpMode {
    List<DcMotorEx> motors = new ArrayList<>();

    @Override
    public void init() {
        RobotMap.getInstance().init(hardwareMap);
        for (HardwareDevice h : RobotMap.getInstance().getDevices()) {
            if (h instanceof DcMotorEx) {
                motors.add((DcMotorEx) h);
            }
        }
        telemetry.addData("Robot Reset", "Press START to reset robot variables and encoders");
    }

    @Override
    public void start() {
        for (DcMotorEx m : motors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        RobotStatus.resetValues();
        Arm.getInstance().resetOffset();
    }

    @Override
    public void loop() {
        requestOpModeStop();
    }
}
