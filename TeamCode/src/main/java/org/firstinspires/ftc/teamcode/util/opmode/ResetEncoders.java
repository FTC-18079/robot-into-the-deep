package org.firstinspires.ftc.teamcode.util.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.teamcode.RobotMap;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Encoder Reset", group = "Util")
public class ResetEncoders extends OpMode {
    List<DcMotorEx> motors = new ArrayList<>();

    @Override
    public void init() {
        RobotMap.getInstance().init(hardwareMap);
        for (HardwareDevice h : RobotMap.getInstance().getDevices()) {
            if (h instanceof DcMotorEx) {
                motors.add((DcMotorEx) h);
            }
        }
    }

    @Override
    public void start() {
        for (DcMotorEx m : motors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    @Override
    public void loop() {
        requestOpModeStop();
    }
}
